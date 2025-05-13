import time
import grovepi
import paho.mqtt.client as mqtt

# config
button_port = 3
mqtt_broker_ip = "localhost"
mqtt_topic = "hospital/gp/available"
loop_delay = 0.1

# MQTT client
client = mqtt.Client()
client.connect(mqtt_broker_ip, 1883, 60)
client.loop_start()

last_state = 0

while True:
    try:
        # Read USR distance val
        new_state = grovepi.digitalRead(button_port)
        if new_state != last_state:
            last_state = new_state
            # Publish to topic
            client.publish(mqtt_topic, str(new_state))
            print(f"Posted: {new_state}")

    except IOError:
        print("Error reading button")

    time.sleep(loop_delay)

