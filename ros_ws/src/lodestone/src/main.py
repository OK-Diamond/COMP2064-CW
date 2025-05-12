#!/usr/bin/env python3
import errno
import json
from enum import Enum

import paho.mqtt.subscribe as subscribe
import paho.mqtt.publish as pub
from paho.mqtt.client import MQTTMessage, Client

import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from dropplot.logger import log_err, log_info, init_log_pub, log_warn
from dropplot.shared import get_sim_init_pose
from dropplot.types import Patient, decode_patient, Pairing, encode_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from locmap.msg import LocMapGoto
from lodestone.msg import State as StateMsg
from dropplot.shared import status_strs

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

USR_THRESHOLD= rospy.get_param("USR_THRESHOLD")

LOC_NAME_ENTRANCE= rospy.get_param("LOC_NAME_ENTRANCE")
LOC_NAME_WAITING_ROOM= rospy.get_param("LOC_NAME_WAITING_ROOM")

IS_SIM: bool= rospy.get_param("IS_SIM")

free_gps: list= [False for _ in range(GP_COUNT)]
waiting_patients: list= []

init_log_pub("LODE")

rospy.init_node('lodestone')
time_of_last= rospy.Time().now()

# The state is internal high-level state, it is basically a linear progression,
#  first the robot is idle (waiting for gp or patient)
#  then will move to the waiting room
#   plus a small delay
#  then will move to the gp
#   plus a small delay
class State(Enum):
    IDLE= 0
    TO_WAITING= 1
    WAITING_AT_PATIENT= 2
    TO_GP= 3
    WAITING_AT_GP= 4

state: State= State.IDLE
# status is the move_base status, so we know what the robot is doing
status: int= GoalStatus.PENDING

current_pairing: Pairing= None

# when we update the state we also send out for bugeyes
def set_state(new_state: State) -> None:
    global state

    if state != new_state:
        log_info(f"Moving from State {state.name} to {new_state.name}")

    state= new_state
    send_new_status_state()

# when we update the status we also send out for bugeyes
def set_status(new_status: int) -> None:
    global status

    if status != new_status:
        log_info(f"Moving from Status {status_to_string(status)} to {status_to_string(new_status)}")

    status= new_status
    send_new_status_state()

def status_to_string(status: int) -> str:
    if status > len(status_strs):
        log_info(f"status_to_string given invalid status. Outside range ({status} > {len(status_strs)})")
        return f"INVALID STATUS VALUE {status}"
    return status_strs[status]

# callback from move_base/status gets new robot status for movement
def status_callback(status_array: GoalStatusArray) -> None:
    global status

    if len(status_array.status_list) == 0:
        set_status(GoalStatus.SUCCEEDED)
        return

    new_status: int= status_array.status_list[-1].status

    set_status(new_status)

    if state == State.IDLE:
        # we only care to update the task when IDLE when a new patient or gp is added, so not from this new status
        return

    # if we have reached the destination, or failed in some way we want to find a new task to do
    if is_terminal_status(new_status):
        update_task()

def send_init_pos() -> None:
    init_pos_pub.publish(get_sim_init_pose())
    log_info("send_init_pos sent AUTO initial position data")

def send_new_status_state() -> None:
    message= StateMsg()
    message.header.stamp= rospy.Time().now()
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

def is_bad_status(status: int) -> bool:
    return  status == GoalStatus.ABORTED   or status == GoalStatus.REJECTED

def is_terminal_status(status: int) -> bool:
    return  status == GoalStatus.PREEMPTED or status == GoalStatus.SUCCEEDED or \
            status == GoalStatus.ABORTED   or status == GoalStatus.REJECTED  or \
            status == GoalStatus.RECALLED

def goto(location_name) -> None:
    location = LocMapGoto()
    location.header.stamp = rospy.Time.now()
    location.header.frame_id = "map"

    location.location_name = location_name

    goal_pub.publish(location)

def gp_callback(decoded: str) -> None:
    global free_gps

    log_info(f"Got decoded {decoded}")
    data: dict= json.loads(decoded)

    try:
        gp_id= data["room"]
        is_ready= data["ready"]

        if gp_id - 1 >= GP_COUNT or gp_id <= 0:
            log_err(f"gp_callback given GP ID that is out of the range. {gp_id - 1} >= {GP_COUNT} or {gp_id} <= 0", errno.ERANGE)
            return

        free_gps[gp_id - 1]= is_ready

        update_task()

        print(f"GP ping with {GP_PREFIX}{gp_id}")
    except ValueError as err:
        log_err(f"gp_callback Unable to parse GP-ID from MQTT message Value: {decoded} Err: {err}", errno.EINVAL)

def usr_callback(decoded: str) -> None:
    try:
        usr_value= float(decoded)

        if usr_value < USR_THRESHOLD:
            pass
    except ValueError as err:
        log_err(f"usr_callback unable to parse USR value from MQTT message Value: {decoded} Err: {err}", errno.EINVAL)

def patient_callback(decoded: str) -> None:
    patient= decode_patient(decoded)

    if patient is None:
        log_err(f"patient_callback unable to decode patient data from message ({decoded})", errno.EINVAL)
        return

    log_info(f"patient_callback received new patient information {patient}")
    waiting_patients.append(patient)

    update_task()

def decode_message(message: MQTTMessage) -> str:
    return message.payload.decode("utf-8")

def on_message(client: Client, userdata, message: MQTTMessage) -> None:
    if message.topic == GP_TOPIC:
        gp_callback(decode_message(message))
    elif message.topic == USR_TOPIC:
        usr_callback(decode_message(message))
    elif message.topic == PATIENT_TOPIC:
        patient_callback(decode_message(message))
    else:
        log_err("on_message received unknown topic message `{message.topic}`", errno.EINVAL)

def find_free_gp_location():
    if not any(free_gps):
        log_info(f"find_free_gp_location was asked for a gp location however there are currently none free")
        return None

    try:
        i= free_gps.index(True)
        free_gps[i]= False
        return GP_PREFIX + str(i + 1)
    except ValueError:
        log_err(f"find_free_gp_location cannot find an index of a free gp even though there is at least one (BAD)", errno.ERANGE)

def post_pairing() -> None:
    pub.single(PAIRING_TOPIC, encode_data(current_pairing))

def update_task() -> None:
    global state, time_of_last, current_pairing

    diff: rospy.Duration= rospy.Time().now() - time_of_last

    if diff.to_sec() < 0.5:
        log_info(f"Would be skipping small diff in update_task. Diff is {diff.to_sec()}")

    time_of_last= rospy.Time().now()

    # just to log bad entries into update_task
    if is_bad_status(status):
        log_warn(f"update_task was called from a bad status update. Found status {status_to_string(status)}", errno.EINVAL)

    # if we are currently doing some action then just return; unless we are idle
    if state != State.IDLE and not is_terminal_status(status):
        log_info("Non terminal state was ignored in update_task")
        return

    # if there are no free gps or no waiting patients then try and go to the entrance
    if state == State.IDLE and (not any(free_gps) or len(waiting_patients) == 0):
        log_info(f"There are no free gps and/or no waiting patients. GP Count: {free_gps.count(True)} Patient Count: {len(waiting_patients)}")

        goto(LOC_NAME_ENTRANCE)
        return

    # here we have a state transition where we have competed the action_lib stuff (it is a terminal status)
    #  and so we need to do the next step depending on the current state
    if state == State.IDLE:
        # if we are idle then we can start from scratch, find a patient and take them to the gp
        set_state(State.TO_WAITING)

        patient: Patient= waiting_patients.pop(0)
        gp_location= find_free_gp_location()

        current_pairing= Pairing(patient, gp_location)

        log_info(f"Created pairing for patient: {patient.name} with {gp_location}")
        post_pairing()

        log_info(f"update_task is now going to waiting room for {patient}")
        goto(LOC_NAME_WAITING_ROOM)
        return
    elif state == State.TO_WAITING:
        # if we were going to the waiting room then we have now arrived
        # we need to now go to the selected gp
        set_state(State.TO_GP)

        gp_location= current_pairing.gp
        log_info(f"update_task has selected gp {gp_location} for patient")
        goto(gp_location)
        return
    elif state == State.TO_GP:
        # if we were going to the gp then we have now arrived
        # we can now go back to idle
        set_state(State.IDLE)
        update_task() # recursive to find next task
        return
    else:
        log_err(f"Found unknown state in update_task. Got state {state.name} ({state.value})", errno.EINVAL)

# sending the initial position should be delayed slightly to wait for the other nodes
#  Not sure if there is a way to wait for a specific node to spin up
if IS_SIM:
    rospy.Timer(rospy.Duration(3), lambda _: send_init_pos(), oneshot= True)

goal_pub= rospy.Publisher(GOTO_TOPIC, LocMapGoto, queue_size= 1)
init_pos_pub= rospy.Publisher(INIT_POSE_TOPIC, PoseWithCovarianceStamped, queue_size= 1)
state_pub= rospy.Publisher(STATE_TOPIC, StateMsg, queue_size= 1)

status_sub= rospy.Subscriber(STATUS_TOPIC, GoalStatusArray, status_callback)

subscribe.callback(on_message, [GP_TOPIC, USR_TOPIC, PATIENT_TOPIC], hostname=MQTT_IP)

rospy.spin()
