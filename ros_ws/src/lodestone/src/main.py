#!/usr/bin/env python3
import rospy
import paho.mqtt.subscribe as subscribe

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from dropplot.shared import get_sim_init_pose
from dropplot.types import Patient, PatientEncoder, decode_patient
from std_msgs.msg import String
from locmap.msg import LocMapGoto
from lodestone.msg import State as StateMsg

# used for the /initialpose
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

from enum import Enum

import json

GP_TOPIC= rospy.get_param("MQTT_GP_TOPIC")
USR_TOPIC= rospy.get_param("MQTT_USR_TOPIC")
PATIENT_TOPIC= rospy.get_param("MQTT_PATIENT_TOPIC")

INIT_POSE_TOPIC= rospy.get_param("INIT_POSE_TOPIC")
STATUS_TOPIC= rospy.get_param("STATUS_TOPIC")
STATE_TOPIC= rospy.get_param("STATE_TOPIC")

GP_PREFIX= rospy.get_param("GP_PREFIX")
GP_COUNT= rospy.get_param("GP_COUNT")

USR_THRESHOLD= rospy.get_param("USR_THRESHOLD")

LOC_NAME_ENTRANCE= rospy.get_param("LOC_NAME_ENTRANCE")
LOC_NAME_WAITING_ROOM= rospy.get_param("LOC_NAME_WAITING_ROOM")

free_gps= [False for _ in range(GP_COUNT)]
waiting_people= []

rospy.init_node('lodestone')

goal_pub= rospy.Publisher("/locmap/goto", LocMapGoto, queue_size= 1)
init_pos_pub= rospy.Publisher(INIT_POSE_TOPIC, PoseWithCovarianceStamped, queue_size= 1)
state_pub= rospy.Publisher(STATE_TOPIC, StateMsg, queue_size= 1)

class State(Enum):
    IDLE= 0
    TO_GP= 1
    TO_WAITING= 2
    WAITING= 3

def set_state(new_state):
    global state
    state= new_state
    send_new_status_state()

def set_status(new_status):
    global status
    status= new_status
    send_new_status_state()

state= State.IDLE
status= GoalStatus.PENDING

status_strs = [""] * (GoalStatus.LOST + 1)
status_strs[GoalStatus.PENDING] = "PENDING"
status_strs[GoalStatus.ACTIVE] = "ACTIVE"
status_strs[GoalStatus.PREEMPTED] = "PREEMPTED"
status_strs[GoalStatus.SUCCEEDED] = "SUCCEEDED"
status_strs[GoalStatus.ABORTED] = "ABORTED"
status_strs[GoalStatus.REJECTED] = "REJECTED"
status_strs[GoalStatus.PREEMPTING] = "PREEMPTING"
status_strs[GoalStatus.RECALLING] = "RECALLING"
status_strs[GoalStatus.RECALLED] = "RECALLED"
status_strs[GoalStatus.LOST] = "LOST"

def status_to_string(status):
    if status > len(status_strs):
        return f"INVALID STATUS VALUE {status}"
    return status_strs[status]

def status_callback(status_array: GoalStatusArray):
    global status

    if len(status_array.status_list) == 0:
        set_status(GoalStatus.SUCCEEDED)
        return

    new_status= status_array.status_list[0].status

    set_status(new_status)
    if is_terminal_status(new_status):
        # update_task()
        pass

status_sub= rospy.Subscriber(STATUS_TOPIC, GoalStatusArray, status_callback)

init_pos_pub.publish(get_sim_init_pose())

def send_new_status_state():
    message= StateMsg()
    message.header.stamp= rospy.Time().now()
    message.state_code= state.value
    message.state_name= state.name
    message.status_code= status
    message.status_name= status_to_string(status)

    message.patients= [patient.name for patient in waiting_people]
    message.free_gps= free_gps

    state_pub.publish(message)

def is_terminal_status(status):
    return  status == GoalStatus.PREEMPTED or status == GoalStatus.SUCCEEDED or \
            status == GoalStatus.ABORTED   or status == GoalStatus.REJECTED  or \
            status == GoalStatus.RECALLED

def goto(location_name):
    location = LocMapGoto()
    location.header.stamp = rospy.Time.now()
    location.header.frame_id = "map"

    location.location_name = location_name

    goal_pub.publish(location)

def gp_callback(message):
    global free_gps

    decoded = message.payload.decode("utf-8")

    try:
        gp_id= int(decoded)
        if gp_id - 1 >= GP_COUNT:
            print(f"GP ID from callback is out of range got id {gp_id}")
            return

        free_gps[gp_id - 1]= True

        # goto(GP_PREFIX + str(gp_id))
        send_new_status_state()
        update_task()

        print(f"GP ping with {GP_PREFIX}{gp_id}")
    except ValueError as err:
        rospy.logerr(f"Unable to parse GP-ID from MQTT message Value: {decoded} Err: {err}")

def usr_callback(message):
    decoded = message.payload.decode("utf-8")

    try:
        usr_value= float(decoded)

        if usr_value < USR_THRESHOLD:
            # print(f"USR ping with {usr_value}")
            pass
    except ValueError as err:
        rospy.logerr(f"Unable to parse USR value from MQTT message Value: {decoded} Err: {err}")

def patient_callback(message):
    decoded_string= message.payload.decode("utf-8")
    patient= decode_patient(decoded_string)

    if patient is None:
        print(f"UNABLE TO ADD PATIENT FROM {message}")
        return

    waiting_people.append(patient)

    update_task()

def on_message(client, userdata, message):
    if message.topic == GP_TOPIC:
        gp_callback(message)
    elif message.topic == USR_TOPIC:
        usr_callback(message)
    elif message.topic == PATIENT_TOPIC:
        patient_callback(message)
    else:
        rospy.logerr(f"Received unknown topic message `{message.topic}`")

def find_free_gp_location():
    if not any(free_gps):
        print("Asked for free gp location but no gps are free")
        return None

    try:
        i= free_gps.index(True) + 1
        return GP_PREFIX + str(i)
    except ValueError:
        print(f"How did we get here? there is a true value in free gps {free_gps}")

def update_task():
    global state

    # if we are currently doing some action then just return
    if not is_terminal_status(status):
        return

    # if there are no free gps or no waiting patients then try and go to the entrance
    if not any(free_gps) or len(waiting_people) == 0:
        set_state(State.IDLE)
        goto(LOC_NAME_ENTRANCE)
        return

    # here we have a state transition where we have competed the action_lib stuff (it is a terminal status)
    #  and so we need to do the next step depending on the current state
    if state == State.IDLE:
        # if we are idle then we can start from scratch, find a patient and take them to the gp
        set_state(State.TO_WAITING)
        goto(LOC_NAME_WAITING_ROOM)
        return
    elif state == State.TO_WAITING:
        # if we were going to the waiting room then we have now arrived
        # we need to now go to a free gp
        set_state(State.TO_GP)
        gp_location= find_free_gp_location()
        goto(gp_location)
        return
    elif state == State.TO_GP:
        # if we were going to the gp then we have now arrived
        # we can now go back to idle
        set_state(State.IDLE)
        return
    else:
        print(f"Found unknown state in update_task got {state}")

MQTT_IP= "10.148.187.223"
subscribe.callback(on_message, [GP_TOPIC, USR_TOPIC, PATIENT_TOPIC], hostname=MQTT_IP)

rospy.spin()
