import time
import grovepi
import paho.mqtt.client as mqtt

# config
usr_d_port = 4
mqtt_broker_ip = "192.168.0.113"
mqtt_topic = "USR"
publish_delay = 0.5

# MQTT client
client = mqtt.Client()
client.connect(mqtt_broker_ip, 1883, 60)
client.loop_start()

while True:
    try:
        # Read USR distance val
        distance = grovepi.ultrasonicRead(usr_d_port)

        # Publish to topic
        client.publish(mqtt_topic, str(distance))
        print(f"Posted: {distance} cm")

    except IOError:
        print("Error reading USR")

    time.sleep(publish_delay)

