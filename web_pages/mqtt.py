'''
MQTT related code
'''

import json
from queue import Queue
from paho.mqtt.client import Client as MQTTClient, MQTTMessage
from multithread_datatypes import ThreadsafeRoomList as RoomList
from common import User, Topics


class MqttManager:
    '''Manages MQTT requests for IoT'''
    BROKER = "localhost"
    PORT = 1883

    def __init__(self, user_queue:Queue, room_list:RoomList, topics:Topics) -> None:
        self.user_queue = user_queue
        self.room_list = room_list
        self.topics = topics

        # Setup client
        client = MQTTClient()
        client.on_connect = self.on_connect
        client.on_message = self.on_message

        # Connect to broker
        client.connect(self.BROKER, self.PORT)
        client.loop_start()  # Start background thread for MQTT
        self.client = client
        print("MQTT client started")

    def process_gp_available(self, payload: dict) -> None:
        '''Process a GP availability message'''
        # Example: {room:3, ready:True}
        if "room" in payload and payload.get("ready", False):
            room_number = payload["room"]
            print(f"GP available in room {room_number}.")

            # Wait for a user
            print(f"Waiting for user... ({room_number})")
            user:User = self.user_queue.get()
            print(f"Please can {user.name} proceed to room {room_number}. The robot will escort you.")
            self.post_robot(user.name, room_number)

    def process_user_register(self, payload:dict) -> None:
        '''Process a user signup message'''
        # Example: {name:"Bob", dob:"2001-03-20", time:98765.4}
        user = User(payload["name"], payload["dob"], payload["time"])
        # Add the user to the queue
        self.user_queue.put(user)
        # No more logic is needed as process_gp_available manages matching users and gps

    def post_robot(self, name:str, room:int) -> None:
        '''Sends a message to the robot via MQTT'''
        data = {
            "action": "collect_user",
            "user_name": name,
            "room": room,
        }
        self.client.publish(self.topics.robot.topic, json.dumps(data))

    def post_user(self, user:User) -> None:
        '''Posts to the user topic'''
        data = {
            "name": user.name,
            "dob": user.dob,
            "time": user.register_time
        }
        self.client.publish(self.topics.user.topic, json.dumps(data))

    def on_connect(self, client:MQTTClient, _userdata, _flags, rc:int) -> None:
        '''Callback when connecting'''
        print(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(self.topics.staff.topic)
        print(f"Subscribed to {self.topics.staff.topic}")

    def on_message(self, _client, _userdata, msg:MQTTMessage):
        '''Callback when receiving a message'''
        try:
            payload = json.loads(msg.payload.decode())  # Converts JSON to dict
            # Datermine whether the message is from a room or from a user
            match msg.topic:
                case self.topics.staff.topic:
                    return self.process_gp_available(payload)
                case self.topics.user.topic:
                    return self.process_user_register(payload)
                case _:
                    raise ValueError(f"Unexpected topic: {msg.topic}")

        except json.JSONDecodeError:
            print(f"Error decoding MQTT message: {msg.payload}")
