#!/usr/bin/env python3
import rospy
import paho.mqtt.subscribe as subscribe

from std_msgs.msg import String
from locmap.msg import LocMapGoto

GP_TOPIC= rospy.get_param("GP_TOPIC")
USR_TOPIC= rospy.get_param("USR_TOPIC")

GP_PREFIX= rospy.get_param('GP_PREFIX')

USR_THRESHOLD= rospy.get_param('USR_THRESHOLD')

free_gps= set()
waiting_people= []

rospy.init_node('lodestone')

goal_pub= rospy.Publisher("/locmap/goto", LocMapGoto, queue_size=1)

def gp_callback(message):
    global free_gps

    decoded = message.payload.decode("utf-8")

    try:
        gp_id= int(decoded)
        free_gps.add(gp_id)

        location= LocMapGoto()
        location.header.stamp= rospy.Time.now()
        location.header.frame_id= "map"

        location.location_name= GP_PREFIX + str(gp_id)

        goal_pub.publish(location)
        print(f"GP ping with {GP_PREFIX}{gp_id}")
    except ValueError as err:
        rospy.logerr(f"Unable to parse GP-ID from MQTT message Value: {decoded} Err: {err}")

def usr_callback(message):
    decoded = message.payload.decode("utf-8")

    try:
        usr_value= float(decoded)

        if usr_value < USR_THRESHOLD:
            print(f"USR ping with {usr_value}")
    except ValueError as err:
        rospy.logerr(f"Unable to parse USR value from MQTT message Value: {decoded} Err: {err}")

def on_message(client, userdata, message):
    if message.topic == GP_TOPIC:
        gp_callback(message)
    elif message.topic == USR_TOPIC:
        usr_callback(message)
    else:
        rospy.logerr(f"Received unknown topic message `{message.topic}`")

subscribe.callback(on_message, [GP_TOPIC, USR_TOPIC], hostname="localhost")

rospy.spin()
