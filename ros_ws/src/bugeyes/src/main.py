#!/usr/bin/env python3

import threading
import time
import tkinter as tk

import paho.mqtt.publish as pub

import rospy
from actionlib_msgs.msg import GoalID
from dropplot.logger import init_log_pub, log_info, LOG_TYPE
from dropplot.shared import get_sim_init_pose
from dropplot.types import Patient, encode_patient
from geometry_msgs.msg import PoseWithCovarianceStamped
from locmap.msg import LocMapLocations, LocMapGoto, LocMapLocation
from lodestone.msg import State, Log

MQTT_IP= rospy.get_param("MQTT_IP")

# just keeping track of the logs generated
logs= []

rospy.init_node("bugeyes_gui", anonymous=True)

# most params have been seperated into ros params
#  this is so that the other nodes can access them as well,
#  makes it easier to make changes (view the launch params.yaml file for them)
INIT_POSE_TOPIC= rospy.get_param("INIT_POSE_TOPIC")
STATE_TOPIC= rospy.get_param("STATE_TOPIC")
LOG_TOPIC= rospy.get_param("LOG_TOPIC")
GOTO_TOPIC= rospy.get_param("GOTO_TOPIC")
LOCATIONS_TOPIC= rospy.get_param("LOCATIONS_TOPIC")
END_GOAL_TOPIC= rospy.get_param("END_GOAL_TOPIC")

GP_COUNT= rospy.get_param("GP_COUNT")
USR_MIN, USR_MAX= rospy.get_param("USR_MIN"), rospy.get_param("USR_MAX")
USR_PING_DURATION= rospy.get_param("USR_PING_DURATION")

MQTT_GPS_TOPIC= rospy.get_param("MQTT_GP_TOPIC")
MQTT_USR_TOPIC= rospy.get_param("MQTT_USR_TOPIC")
MQTT_PATIENT_TOPIC= rospy.get_param("MQTT_PATIENT_TOPIC")

# this publisher is for directing the robot via locmap (using string names)
goto_pub= rospy.Publisher(GOTO_TOPIC, LocMapGoto, queue_size=1)
# this publisher is for cancelling the current goal (maily for debugging)
end_goal_pub= rospy.Publisher(END_GOAL_TOPIC, GoalID, queue_size=1)
# this publisher is for sending the initial pose information of the robot for localisation
init_pose_pub= rospy.Publisher(INIT_POSE_TOPIC, PoseWithCovarianceStamped, queue_size=1)

# The logger needs to be initialised before use
init_log_pub("BUGI")

# root of the tkinter window
root= tk.Tk()

# these variables are for the filtering system, just to make it easier to see errors
show_info, show_warn, show_err= tk.BooleanVar(value= True), tk.BooleanVar(value= True), tk.BooleanVar(value= True)
paused= tk.BooleanVar(value= False)
search_term= tk.StringVar()
# this is for pausing logs, tracking which ids are the last
last_pause_ids= {} #: dict[str, int] PY3.8 >:(
# just for error checking when making this; if logs appear out of order then it will show an error (red id)
last_ids= {} #: dict[str, int]

# global ui elements for easy access
log_text: tk.Text
left: tk.Frame
right: tk.Frame
loc_frame: tk.Frame
gp_frame: tk.Frame
state_label: tk.Label
patient_label: tk.Label
usr_slider: tk.Scale
gp_buttons= []#: list[tk.Button]
loc_buttons= []#: list[tk.Button]

patient_count: int= 0
usr_ping_timer: int= 0

# create a grid widget, with some base configuration
def gridget(widget, row, col, **kwargs):
    widget.grid(row= row, column= col, padx= 5, pady= 10, sticky= "nsew", **kwargs)
    return widget

# this function creates all the ui elements
def create_ui():
    global log_text, left, right

    root.title("Bug eyes")
    root.configure(background= "grey")
    root.minsize(640, 480)
    root.maxsize(1920, 1080)

    main= tk.Frame(root)
    main.pack(fill= tk.BOTH, expand= True)

    # the left is the control panel, it will contain the interaction buttons
    left= tk.Frame(main)
    left.pack(side= tk.LEFT, fill= tk.BOTH, expand= False)

    # right is for the log view
    right= tk.Frame(main, bg="black")
    right.pack(side= tk.RIGHT, fill= tk.BOTH, expand= True)

    def create_logs_ui():
        global log_text

        log_label= tk.Label(right, text= "Logs", bg= "black")
        log_label.pack(fill= tk.X)

        filter_frame= tk.Frame(right, bg= "black")
        filter_frame.pack(fill= tk.X, expand= False)

        def create_check(text, var, lam= lambda: reset_log()):
            return tk.Checkbutton(
                filter_frame, text= text, variable= var, bg= "black",
                fg= "grey", selectcolor= "black", command= lam
            ).pack(side= tk.LEFT, padx= 2, pady= 3)

        create_check("Info", show_info)
        create_check("Warn", show_warn)
        create_check("Error", show_err)

        create_check("Pause", paused, lam= lambda: toggle_pause())

        search_entry= tk.Entry(filter_frame, textvariable= search_term)
        search_entry.pack(side= tk.RIGHT, padx= 3, pady= 5)
        search_entry.bind("<KeyRelease>", lambda _: reset_log)

        log_text= tk.Text(right, state= 'disabled', wrap= 'word', bg= "black", fg= "lightgreen")
        log_text.pack(side= tk.LEFT, fill= tk.BOTH, expand= True)

        # these let the text inserted have different colors
        log_text.tag_configure(LOG_TYPE.INFO.name, foreground= "lightgreen")
        log_text.tag_configure(LOG_TYPE.WARN.name, foreground= "purple")
        log_text.tag_configure(LOG_TYPE.ERR.name, foreground= "red")

        scrollbar= tk.Scrollbar(right, command= log_text.yview)
        scrollbar.pack(side= tk.RIGHT, fill= tk.Y)
        log_text.config(yscrollcommand= scrollbar.set)

    def create_actions_ui():
        global usr_slider, gp_frame, gp_buttons, loc_frame, loc_buttons, state_label, patient_label

        usr_label = gridget(tk.Label(left, text= "USR", height= 3), 0, 0)
        usr_slider = gridget(tk.Scale(left, from_= USR_MIN, to= USR_MAX, orient= tk.HORIZONTAL, resolution= 0.05, length= 300),
                             0, 1)
        usr_ping = gridget(tk.Button(left, text= "PING", height= 1, width= 4), 0, 2)

        gp_frame = tk.Frame(left)
        gridget(gp_frame, 1, 0, columnspan= GP_COUNT + 1)
        gp_label = gridget(tk.Label(gp_frame, text= "GPS", height= 3), 0, 0)
        gp_buttons = [
            gridget(tk.Button(gp_frame, text= f"GP-{i + 1}", width= 5,
                              command= lambda i=i: pub.single(MQTT_GPS_TOPIC, i + 1, hostname= MQTT_IP)), 0, i + 1)
            for i in range(GP_COUNT)
        ]

        loc_frame = tk.Frame(left)
        gridget(loc_frame, 2, 0, columnspan= GP_COUNT + 1)
        locations_label = gridget(tk.Label(loc_frame, text= "Locations", height= 3), 0, 0)
        loc_buttons = [

        ]

        action_frame = tk.Frame(left)
        gridget(action_frame, 3, 0, columnspan= GP_COUNT + 1)
        action_label = gridget(tk.Label(action_frame, text= "Actions", height= 3), 0, 0)

        end_goal_button = gridget(tk.Button(action_frame, text= "End goal", command= lambda: end_goal_pub.publish(GoalID())),
                                  0, 1)

        def send_init_pos():
            init_pose_pub.publish(get_sim_init_pose())
            log_info("bugeyes/send_init_pos sent initial position data")

        send_init_pose_button = gridget(tk.Button(action_frame, text= "Send Init pose", command= lambda: send_init_pos()), 0,
                                        2)
        def send_patient_data():
            global patient_count

            pub.single(MQTT_PATIENT_TOPIC,
                       encode_patient(Patient(f"Paul the {patient_count}erdnd", f"10.{patient_count}.2025")),
                       hostname= MQTT_IP)
            patient_count += 1

        add_person_button = gridget(tk.Button(action_frame, text= "Add waiting", command= send_patient_data), 0, 3)

        info_frame = tk.Frame(left)
        gridget(info_frame, 4, 0, columnspan= GP_COUNT + 1)
        state_label = gridget(tk.Label(info_frame, text= "State: `IDLE` Status: `NONE` at 0"), 0, 0)
        patient_label = gridget(tk.Label(info_frame, text= "Patients: "), 1, 0)

    create_logs_ui()
    create_actions_ui()

# just to decide if a log should be displayed (based on the filtering)
def should_add_log(type: LOG_TYPE, message: str) -> bool:
    if type == LOG_TYPE.INFO and not show_info.get():
        return False
    if type == LOG_TYPE.WARN and not show_warn.get():
        return False
    if type == LOG_TYPE.ERR and not show_err.get():
        return False
    if search_term.get().lower() not in message.lower():
        return False

    return True

# add a log to the text panel with highlighting
def insert_log(type: LOG_TYPE, log: Log, failed_id: bool) -> None:
    errcode_text= "" if type == LOG_TYPE.INFO else f"({log.errno})"
    log_text.insert(tk.END, f"{log.source}#{log.id:03}", LOG_TYPE.ERR.name if failed_id else "")
    log_text.insert(tk.END, f" {type.name}", type.name)
    log_text.insert(tk.END, f"{errcode_text}: {log.message}\n")

# potentially add a log (based on filtering) to the end of the log text
def append_log(type: LOG_TYPE, log: Log) -> None:
    global last_ids
    failed_id = last_ids.get(log.source, -1) >= log.id
    last_ids[log.source]= log.id

    if paused.get():
        return

    if not should_add_log(type, log.message):
        return

    log_text.config(state='normal')

    insert_log(type, log, failed_id)

    log_text.see(tk.END)
    log_text.config(state='disabled')

def toggle_pause() -> None:
    global last_pause_ids

    if not paused.get():
        reset_log()

# clear the logs text and redraw all logs based on filtering
def reset_log() -> None:
    log_text.config(state='normal')
    log_text.delete(1.0, tk.END)

    global last_ids
    last_ids.clear()
    for type, log in logs:
        failed_id= last_ids.get(log.source, -1) >= log.id
        last_ids[log.source]= log.id

        lp_id= last_pause_ids.get(log.source, -1)
        if paused.get() and lp_id > log.id:
            continue

        last_pause_ids[log.source]= max(log.id, lp_id)

        if not should_add_log(type, log.message):
            continue

        insert_log(type, log, failed_id)

    log_text.see(tk.END)
    log_text.config(state= 'disabled')

# callback for when a log is received, gets added to the array and appended to text
def log_callback(log: Log) -> None:
    if log.errno == 0:
        type= LOG_TYPE.INFO
    elif log.errno < 0:
        type= LOG_TYPE.ERR
    elif log.errno > 0:
        type= LOG_TYPE.WARN
    else: type= LOG_TYPE.NONE

    logs.append((type, log))
    append_log(type, log)

# sending a goal location to the locmap api
def send_goal(location: LocMapLocation) -> None:
    goal= LocMapGoto()
    goal.location_name= location.name
    goal.header.stamp= rospy.Time.now()
    goal.header.frame_id= "map"

    goto_pub.publish(goal)

# when new locations are posted from locmap
def locations_callback(locations: LocMapLocations) -> None:
    global loc_buttons, gp_buttons, loc_frame, gp_frame

    loc_buttons.clear()
    gp_buttons.clear()

    non_gps= sorted(filter(lambda loc: not loc.name.startswith("GP-"), locations.locations), key= lambda loc: loc.name)
    gps= sorted(filter(lambda loc: loc.name.startswith("GP-"), locations.locations), key= lambda loc: loc.name)

    loc_buttons= [
        gridget(tk.Button(loc_frame, text= non_gps[loc_i].name, command= lambda loc=non_gps[loc_i]: send_goal(loc)), 0, loc_i + 1)
        for loc_i in range(len(non_gps))
    ]

    gp_buttons= [
        gridget(tk.Button(gp_frame, text= gps[loc_i].name, command= lambda loc=gps[loc_i]: pub.single(MQTT_GPS_TOPIC, loc.name[3:], hostname=MQTT_IP)), 0, loc_i + 1)
        for loc_i in range(len(gps))
    ]

# when the state from lodestone is updated
def state_callback(state: State) -> None:
    global state_label, patient_label, gp_buttons

    state_label.config(text= f"State: `{state.state_name}` Status: `{state.status_name}` at {state.header.stamp}")

    for i, v in enumerate(state.free_gps):
        if not v:
            gp_buttons[i].config(bg= "grey")
        else:
            gp_buttons[i].config(bg= "green")

    patient_label.config(text= f"Patients: {state.patients}")

# sending out usr data for door
def mqtt_loop() -> None:
    global usr_slider

    while True:
        pub.single(MQTT_USR_TOPIC, usr_slider.get(), hostname= MQTT_IP)

        global usr_ping_timer
        usr_ping_timer= USR_PING_DURATION
        while usr_ping_timer > 0:
            usr_ping_timer -= 0.1
            time.sleep(0.1)

create_ui()

if loc_frame is None or gp_frame is None:
    rospy.logwarn("UI not yet initialized, skipping locations_callback")

rospy.Subscriber(LOCATIONS_TOPIC, LocMapLocations, locations_callback)
state_sub= rospy.Subscriber(STATE_TOPIC, State, state_callback)
log_sub= rospy.Subscriber(LOG_TOPIC, Log, log_callback)

mqtt_thread= threading.Thread(target= mqtt_loop, daemon= True)
mqtt_thread.start()

root.mainloop()
