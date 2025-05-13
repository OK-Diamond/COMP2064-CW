#!/usr/bin/env python3
"""Lodestone is the state machine; using information from other ros nodes and mqtt to direct
    the robot"""

import errno
import json
from enum import Enum

import paho.mqtt.publish as pub
from paho.mqtt import subscribe
from paho.mqtt.client import MQTTMessage, Client

import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from dropplot.logger import log_err, log_info, init_log_pub, log_warn
from dropplot.shared import get_sim_init_pose, status_strs
from dropplot.types import Patient, decode_patient, Pairing, encode_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from locmap.msg import LocMapGoto
from lodestone.msg import State as StateMsg

# most params have been seperated into ros params
#  this is so that the other nodes can access them as well, (e.g. lodestone and bugeyes)
#  makes it easier to make changes (view the launch params.yaml file for them)
MQTT_IP= rospy.get_param("MQTT_IP")
GP_TOPIC= rospy.get_param("MQTT_GP_TOPIC")
USR_TOPIC= rospy.get_param("MQTT_USR_TOPIC")
PATIENT_TOPIC= rospy.get_param("MQTT_PATIENT_TOPIC")
PAIRING_TOPIC= rospy.get_param("MQTT_PAIRING_TOPIC")

INIT_POSE_TOPIC= rospy.get_param("INIT_POSE_TOPIC")
STATUS_TOPIC= rospy.get_param("STATUS_TOPIC")
STATE_TOPIC= rospy.get_param("STATE_TOPIC")
GOTO_TOPIC= rospy.get_param("GOTO_TOPIC")

GP_PREFIX= rospy.get_param("GP_PREFIX")
GP_COUNT= rospy.get_param("GP_COUNT")

WAITING_MAX_TIME: float= rospy.get_param("WAITING_TIME_MAX")

USR_THRESHOLD= rospy.get_param("USR_THRESHOLD")

LOC_NAME_ENTRANCE= rospy.get_param("LOC_NAME_ENTRANCE")
LOC_NAME_WAITING_ROOM= rospy.get_param("LOC_NAME_WAITING_ROOM")

IS_SIM: bool= rospy.get_param("IS_SIM")

# data on gps and patients (updated from MQTT)
free_gps: list= [False for _ in range(GP_COUNT)]
waiting_patients: list= []
current_pairing: Pairing= None

# to use the logger (log_(info/warn/err)) we need to initialise it
init_log_pub("LODE")

# to use the ros pubs/subs we need to initialise the node
rospy.init_node('lodestone')

# ros sub/pub nodes
goal_pub: rospy.Publisher
init_pos_pub: rospy.Publisher
state_pub: rospy.Publisher
status_sub: rospy.Subscriber

# the last time we updated the state
time_of_last_update= rospy.Time.now()
# how long we have been waiting (used in WAITING_AT_ states)
waiting_time_end: rospy.Time= rospy.Time.now()

# The state is internal high-level state, it is basically a linear progression,
#  first the robot is idle (waiting for gp or patient)
#  then will move to the waiting room
#   plus a small delay
#  then will move to the gp
#   plus a small delay
class State(Enum):
    """High level state ENUM for current robot goal"""
    IDLE= 0
    TO_WAITING= 1
    WAITING_AT_PATIENT= 2
    TO_GP= 3
    WAITING_AT_GP= 4

state: State= State.IDLE
# status is the move_base status, so we know what the robot is doing
status: int= GoalStatus.PENDING

def main():
    """Entry function sets up the ros and mqtt pubs/subs and sends initial sim pose data"""
    global goal_pub, init_pos_pub, state_pub, status_sub

    # sending the initial position should be delayed slightly to wait for the other nodes
    #  Not sure if there is a way to wait for a specific node to spin up
    if IS_SIM:
        rospy.Timer(rospy.Duration(3), lambda _: publish_init_pose_message(), oneshot= True)

    goal_pub= rospy.Publisher(GOTO_TOPIC, LocMapGoto, queue_size= 1)
    init_pos_pub= rospy.Publisher(INIT_POSE_TOPIC, PoseWithCovarianceStamped, queue_size= 1)
    state_pub= rospy.Publisher(STATE_TOPIC, StateMsg, queue_size= 1)

    status_sub= rospy.Subscriber(STATUS_TOPIC, GoalStatusArray, status_callback)

    subscribe.callback(on_message, [GP_TOPIC, USR_TOPIC, PATIENT_TOPIC], hostname=MQTT_IP)

    rospy.spin()

def update_task() -> None:
    """Update the internal state potentially to a new state
        This should be called when something new happens
         e.g. new patient is registered, or gp status update"""
    global time_of_last_update

    diff: rospy.Duration= rospy.Time.now() - time_of_last_update

    if diff.to_sec() < 0.2:
        log_info(f"Small diff in update_task. Diff is {diff.to_sec()}")

    time_of_last_update= rospy.Time.now()

    # just to log bad entries into update_task
    if is_bad_status(status):
        log_warn("update_task was called from a bad status update. Found status "
                 f"{status_to_string(status)}", errno.EINVAL)

    update_state(state, is_terminal_status(status))

def enter_state(state: State):
    """Called when a transition from old_state -> state occurs
        This function does any setup for the new state
        e.g. directing the robot to a location, or setting up timers"""
    global waiting_time_end

    if state == State.IDLE:
        publish_goto_message(LOC_NAME_ENTRANCE)
        # recursive to find next task
        update_task()
    elif state == State.TO_WAITING:
        publish_goto_message(LOC_NAME_WAITING_ROOM)
    elif state == State.WAITING_AT_GP:
        waiting_time_end= rospy.Time.now() + rospy.Duration(WAITING_MAX_TIME)
    elif state == State.TO_GP:
        publish_goto_message(current_pairing.gp)
        log_info(f"update_state previously selected gp {current_pairing.gp} for "
                 f"patient {current_pairing.patient.name}")
    elif state == State.WAITING_AT_PATIENT:
        waiting_time_end= rospy.Time.now() + rospy.Duration(WAITING_MAX_TIME)
    else:
        log_err(f"Found unknown state in enter_state. Got state {state.name} ({state.value})",
                errno.EINVAL)

def update_state(state: State, terminal_status: bool):
    """Called to update the current state
        This function checks if there is a state transition and updates counters"""
    if state == State.IDLE:
        if any(free_gps) and len(waiting_patients) > 0:
            create_current_pairing()
            publish_pairing_message()
            to_next_state()
    elif state in (State.TO_GP, State.TO_WAITING):
        if terminal_status:
            to_next_state()
    elif state == State.WAITING_AT_PATIENT:
        # here we are just waiting in the waiting room for the patient
        if (waiting_time_end - rospy.Time.now()).to_sec() > 0:
            return

        to_next_state()
    elif state == State.WAITING_AT_GP:
        if (waiting_time_end - rospy.Time.now()).to_sec() > 0:
            return

        # if we're waiting at the gp then we can now go back to idle
        to_next_state()
    else:
        log_err(f"Found unknown state in update_state. Got state {state.name} ({state.value})",
                errno.EINVAL)

def to_next_state() -> None:
    """Sets the state to the next state in the linear transition"""
    set_state(State((state.value + 1) % len(State)))

def set_state(new_state: State) -> None:
    """Set the state to a new state
     All modification of state should be through here as it updates bugeyes and calls enter_state"""
    global state

    if state != new_state:
        # as enter_state can call set_state we need to do all the
        #  state updating before calling it; to not overwrite
        old_state= state
        state= new_state
        publish_state_message()

        log_info(f"Moving from State {old_state.name} to {new_state.name}")
        enter_state(new_state)
        return

    state= new_state
    publish_state_message()

def set_status(new_status: int) -> None:
    """Set the status to a new status
     All modification of status should be through here as it updates bugeyes"""
    global status

    if status != new_status:
        log_info(f"Moving from Status {status_to_string(status)} to {status_to_string(new_status)}")

    status= new_status
    publish_state_message()

# callback from move_base/status gets new robot status for movement
def status_callback(status_array: GoalStatusArray) -> None:
    """Callback from move_base/status
        Attributes
        ----------
        status_array: GoalStatusArray
            This is the list of goals that the move_base package has been given and their
             corresponding status; we usually only care about the last one"""
    # if there haven't been any goals then there will be no status provided
    #  just set it to success for logic elsewhere
    if len(status_array.status_list) == 0:
        set_status(GoalStatus.SUCCEEDED)
        return

    # getting the LAST goal's status (they are stored all together in a list)
    new_status: int= status_array.status_list[-1].status

    set_status(new_status)

    if state == State.IDLE:
        # we only care to update the task when IDLE if a new patient or gp is added, so not from
        #  this new status function
        return

    # if we have reached the destination, or failed in some way we want to find a new task to do
    if is_terminal_status(new_status):
        update_task()

def publish_init_pose_message() -> None:
    """Sending the initial pose for the simulator version of the robot for localisation"""
    init_pos_pub.publish(get_sim_init_pose())
    log_info("send_init_pos sent AUTO initial position data")

def publish_state_message() -> None:
    """Publish a new state message for a node like bugeyes this is a summary of lodestone data
        it is only published when a new state or status appears"""
    message= StateMsg()
    message.header.stamp= rospy.Time.now()
    message.state_code= state.value
    message.state_name= state.name
    message.status_code= status
    message.status_name= status_to_string(status)

    message.patients= [patient.name for patient in waiting_patients]
    message.free_gps= free_gps

    if current_pairing is not None:
        message.paired_patient= current_pairing.patient.name
        message.paired_gp= current_pairing.gp

    state_pub.publish(message)

def publish_goto_message(location_name: str) -> None:
    """Publish a locmap goto message for the location
        Locations are verified by locmap and may be rejected if unknown
        Use the locmap locations publisher to get the locations known"""
    location = LocMapGoto()
    location.header.stamp = rospy.Time.now()
    location.header.frame_id = "map"

    location.location_name = location_name

    goal_pub.publish(location)

def gp_callback(decoded: str) -> None:
    """Callback for when a gp sends information about their readiness"""
    data: dict= json.loads(decoded)

    try:
        gp_id= data["room"]
        is_ready= data["ready"]

        if gp_id - 1 >= GP_COUNT or gp_id <= 0:
            log_err(f"gp_callback given GP ID that is out of the range. {gp_id - 1} >= {GP_COUNT} "
                    f"or {gp_id} <= 0", errno.ERANGE)
            return

        free_gps[gp_id - 1]= is_ready
        log_info(f"gp_callback received information that GP {gp_id - 1} is "
                 f"{'free' if is_ready else 'not free'}")

        update_task()
    except ValueError as err:
        log_err(f"gp_callback Unable to parse GP-ID from MQTT message Value: {decoded} Err: {err}",
                errno.EINVAL)

def usr_callback(decoded: str) -> None:
    """Callback for the USR data
        This data is un-used by lodestone but errors are logged"""
    try:
        usr_value= float(decoded)

        if usr_value < USR_THRESHOLD:
            pass
    except ValueError as err:
        log_err(f"usr_callback unable to parse USR value from MQTT message Value: {decoded} "
                f"Err: {err}", errno.EINVAL)

def patient_callback(decoded: str) -> None:
    """Callback for when a patient is registered
        decoded is the utf-8 decoded message payload"""
    patient= decode_patient(decoded)

    if patient is None:
        log_err(f"patient_callback unable to decode patient data from message ({decoded})",
                errno.EINVAL)
        return

    log_info(f"patient_callback received new patient information {patient}")
    waiting_patients.append(patient)

    update_task()

def on_message(client: Client, userdata, message: MQTTMessage) -> None:
    """Callback for all MQTT messages"""
    if message.topic == GP_TOPIC:
        gp_callback(decode_message(message))
    elif message.topic == USR_TOPIC:
        usr_callback(decode_message(message))
    elif message.topic == PATIENT_TOPIC:
        patient_callback(decode_message(message))
    else:
        log_err("on_message received unknown topic message `{message.topic}`", errno.EINVAL)

def decode_message(message: MQTTMessage) -> str:
    """Decode an MQTT message payload as utf-8"""
    return message.payload.decode("utf-8")

def find_free_gp_location():
    """Based on the free gp array find the first available one
        returns the location as a formatted locmap location"""
    if not any(free_gps):
        log_info("find_free_gp_location was asked for a gp location however there are currently "
                 "none free")
        return None

    try:
        i= free_gps.index(True)
        free_gps[i]= False
        return GP_PREFIX + str(i + 1)
    except ValueError:
        log_err(f"find_free_gp_location cannot find an index of a free gp even though there "
                f"is at least one (BAD)", errno.ERANGE)
        return None

def create_current_pairing() -> None:
    """Set the current pairing to a free gp and registered patient pair"""
    global current_pairing

    if len(waiting_patients) == 0 or not any(free_gps):
        log_err(f"create_current_pairing called without any free gps ({any(free_gps)}) "
                f"and/or registered patients {len(waiting_patients)}", errno.EINVAL)
        return

    patient: Patient = waiting_patients.pop(0)
    gp_location = find_free_gp_location()

    current_pairing = Pairing(patient, gp_location)

    log_info(f"Created pairing for patient: {patient.name} with {gp_location}")

def publish_pairing_message() -> None:
    """Post the current pairing to MQTT"""
    pub.single(PAIRING_TOPIC, encode_data(current_pairing))

def is_bad_status(status: int) -> bool:
    """move_base statuses can be reached when the robot is unable to reach a goal
        or if something goes wrong; this function checks for those statuses"""
    return status in (GoalStatus.ABORTED, GoalStatus.REJECTED)

def is_terminal_status(status: int) -> bool:
    """move_base statuses can be reached when the robot has completed (or failed to complete)
        a goal; this function checks for those statuses"""
    return status in (GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                      GoalStatus.REJECTED , GoalStatus.RECALLED                      )

def status_to_string(status: int) -> str:
    """Helper function to get a string version of the status (they are just int consts)"""
    if status > len(status_strs):
        log_info("status_to_string given invalid status. Outside range"
                 f" ({status} > {len(status_strs)})")
        return f"INVALID STATUS VALUE {status}"
    return status_strs[status]

main()
