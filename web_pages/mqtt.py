'''
MQTT related code
'''

import json
from queue import Queue
from time import time
from paho.mqtt.client import Client as MQTTClient, MQTTMessage
from multithread_datatypes import ThreadsafeList as TList
from common import User, Topics


class MqttManager:
    '''Manages MQTT requests for IoT'''
    BROKER = "0.0.0.0"
    PORT = 1883

    def __init__(self, user_queue:Queue, room_list:TList, topics:Topics, message_queue=TList()) -> None:
        self.user_queue = user_queue
        self.room_list = room_list
        self.topics = topics
        self.message_queue: TList = message_queue

        # Setup client
        client = MQTTClient()
        client.on_connect = self.on_connect
        client.on_message = self.on_message

        # Connect to broker
        client.connect(self.BROKER, self.PORT)
        client.loop_start()  # Start background thread for MQTT
        self.client = client
        print("MQTT client started")

    def process_pairing(self, payload: dict) -> None:
        '''Process when a GP and patient have been paried together'''
        if "gp" in payload and "patient" in payload:
            room_number: int= payload["gp"]

            user= payload["patient"]
            message_text = f"Please can {user['name']} proceed to room {room_number}. The robot will escort you."
            print(f"Please can {user['name']} proceed to room {room_number}. The robot will escort you.")

            # Add the message to the queue with current timestamp
            if self.message_queue is not None:
                self.message_queue.add({
                    'text': message_text,
                    'timestamp': time()
                })
        else:
            print("NOT VALID PAYLOAD")

    def post_user(self, user:User) -> None:
        '''Posts to the user topic'''
        data = {
            "name": user.name,
            "dob": user.dob,
            "time": user.register_time
        }
        self.client.publish(self.topics.user.topic, json.dumps(data))

    def post_gp_available(self, room:int, status:bool) -> None:
        '''Posts to the GP available topic'''
        data = {
            "room": room,
            "ready": status
        }
        self.client.publish(self.topics.staff.topic, json.dumps(data))

    def on_connect(self, client:MQTTClient, _userdata, _flags, rc:int) -> None:
        '''Callback when connecting'''
        print(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(self.topics.staff.topic)
        print(f"Subscribed to {self.topics.staff.topic}")
        client.subscribe(self.topics.user.topic)
        print(f"Subscribed to {self.topics.user.topic}")
        client.subscribe(self.topics.pairing.topic)
        print(f"Subscribed to {self.topics.pairing.topic}")


    def on_message(self, _client, _userdata, msg:MQTTMessage):
        '''Callback when receiving a message'''
        payload = json.loads(msg.payload.decode())  # Converts JSON to dict
        # Datermine whether the message is from a room or from a user
        if msg.topic == self.topics.pairing.topic:
            return self.process_pairing(payload)
        else:
            print(f"Unexpected topic: {msg.topic}")
