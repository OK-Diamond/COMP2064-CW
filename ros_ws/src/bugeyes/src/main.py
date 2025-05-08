#!/usr/bin/env python3

import threading
import tkinter as tk
import paho.mqtt.publish as pub
import paho.mqtt.client as client
import time
import rospy
from dropplot.shared import get_sim_init_pose
from dropplot.types import Patient, PatientEncoder, encode_patient
from geometry_msgs.msg import PoseWithCovarianceStamped
from locmap.msg import LocMapLocations, LocMapLocation, LocMapGoto
from actionlib_msgs.msg import GoalID

# from localhost
MQTT_IP="10.148.187.223"

import json

from lodestone.msg import State

rospy.init_node("bugeyes_gui", anonymous=True)

INIT_POSE_TOPIC= rospy.get_param("INIT_POSE_TOPIC")
STATE_TOPIC= rospy.get_param("STATE_TOPIC")

goto_pub= rospy.Publisher("locmap/goto", LocMapGoto, queue_size=1)
end_goal_pub= rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
init_pose_pub= rospy.Publisher(INIT_POSE_TOPIC, PoseWithCovarianceStamped, queue_size=1)

# not sure how much this certainty will affect it finding the complete position, but it is mostly accurate
root= tk.Tk()

root.title("Bug eyes")
root.configure(background="grey")
root.minsize(640, 480)
root.maxsize(1920, 1080)

GP_COUNT= rospy.get_param("GP_COUNT")
USR_MIN, USR_MAX= rospy.get_param("USR_MIN"), rospy.get_param("USR_MAX")
USR_PING_DURATION= rospy.get_param("USR_PING_DURATION")

MQTT_GPS_TOPIC= rospy.get_param("MQTT_GP_TOPIC")
MQTT_USR_TOPIC= rospy.get_param("MQTT_USR_TOPIC")
MQTT_PATIENT_TOPIC= rospy.get_param("MQTT_PATIENT_TOPIC")

usr_ping_timer= 0

def gridget(widget, row, col, **kwargs):
    widget.grid(row=row, column=col, padx=5, pady=10, sticky="nsew", **kwargs)
    return widget

usr_label= gridget(tk.Label(root, text="USR", height= 3), 0, 0)
usr_slider= gridget(tk.Scale(root, from_=USR_MIN, to=USR_MAX, orient=tk.HORIZONTAL, resolution=0.05, length=300), 0, 1)
usr_ping= gridget(tk.Button(root, text="PING", height=1, width=4), 0, 2)

gp_frame= tk.Frame(root)
gridget(gp_frame, 1, 0, columnspan=GP_COUNT + 1)
gp_label= gridget(tk.Label(gp_frame, text= "GPS", height= 3), 0, 0)
gp_buttons= [
    gridget(tk.Button(gp_frame, text= f"GP-{i + 1}", width= 5, command= lambda i=i: pub.single(MQTT_GPS_TOPIC, i + 1, hostname=MQTT_IP)), 0, i + 1)
    for i in range(GP_COUNT)
]

loc_frame= tk.Frame(root)
gridget(loc_frame, 2, 0, columnspan= GP_COUNT + 1)
locations_label= gridget(tk.Label(loc_frame, text= "Locations", height= 3), 0, 0)
loc_buttons= [

]

def send_goal(location):
    goal= LocMapGoto()
    goal.location_name= location.name
    goal.header.stamp= rospy.Time.now()
    goal.header.frame_id= "map"

    goto_pub.publish(goal)

def locations_callback(locations: LocMapLocations):
    global loc_buttons, gp_buttons
    loc_buttons.clear()
    gp_buttons.clear()

    non_gps= sorted(filter(lambda loc: not loc.name.startswith("GP-"), locations.locations), key=lambda loc: loc.name)
    gps= sorted(filter(lambda loc: loc.name.startswith("GP-"), locations.locations), key=lambda loc: loc.name)

    loc_buttons= [
        gridget(tk.Button(loc_frame, text=non_gps[loc_i].name, command=lambda loc=non_gps[loc_i]: send_goal(loc)), 0, loc_i + 1)
        for loc_i in range(len(non_gps))
    ]

    gp_buttons= [
        gridget(tk.Button(gp_frame, text= gps[loc_i].name, command= lambda loc=gps[loc_i]: pub.single(MQTT_GPS_TOPIC, loc.name[3:], hostname=MQTT_IP)), 0, loc_i + 1)
        for loc_i in range(len(gps))
    ]

rospy.Subscriber('locmap/locations', LocMapLocations, locations_callback)

action_frame= tk.Frame(root)
gridget(action_frame, 3, 0, columnspan= GP_COUNT + 1)
action_label= gridget(tk.Label(action_frame, text="Actions", height= 3), 0, 0)

end_goal_button= gridget(tk.Button(action_frame, text="End goal", command=lambda: end_goal_pub.publish(GoalID())), 0, 1)
send_init_pose_button= gridget(tk.Button(action_frame, text="Send Init pose", command= lambda: init_pose_pub.publish(get_sim_init_pose())), 0, 2)

patient_count= 0
def send_patient_data():
    global patient_count

    pub.single(MQTT_PATIENT_TOPIC, encode_patient(Patient(f"Paul the {patient_count}erdnd", "A good reason")), hostname=MQTT_IP)
    patient_count+= 1

add_person_button= gridget(tk.Button(action_frame, text="Add waiting", command=send_patient_data),0, 3)

info_frame= tk.Frame(root)
gridget(info_frame, 4, 0, columnspan= GP_COUNT + 1)
state_label= gridget(tk.Label(info_frame, text="State: `IDLE` Status: `NONE` at 0"), 0, 0)
patient_label= gridget(tk.Label(info_frame, text="Patients: "), 1, 0)

def state_callback(state: State):
    state_label.config(text=f"State: `{state.state_name}` Status: `{state.status_name}` at {state.header.stamp}")

    for i, v in enumerate(state.free_gps):
        if not v:
            gp_buttons[i].config(bg="grey")
        else:
            gp_buttons[i].config(bg="green")

    patient_label.config(text=f"Patients: {state.patients}")

state_sub= rospy.Subscriber(STATE_TOPIC, State, state_callback)

def mqtt_loop():
    while True:
        pub.single(MQTT_USR_TOPIC, usr_slider.get(), hostname=MQTT_IP)

        global usr_ping_timer
        usr_ping_timer= USR_PING_DURATION
        while usr_ping_timer > 0:
            usr_ping_timer -= 0.1
            time.sleep(0.1)

mqtt_thread= threading.Thread(target=mqtt_loop, daemon=True)
mqtt_thread.start()

root.mainloop()

