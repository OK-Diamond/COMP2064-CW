'''
MQTT related code
'''
import asyncio
import json
import threading
from queue import Queue
from time import time

import websockets
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

        # websockets for sending usr pings
        self.web_loop= asyncio.new_event_loop()
        self.web_thread= threading.Thread(
            target= self.setup_websocket,
            daemon= True
        )
        self.web_thread.start()

        self.sock_conns = set()

    async def sock_conn_handler(self, sock_conn):
        """Called when a new socket connects"""
        self.sock_conns.add(sock_conn)
        try:
            async for _ in sock_conn:
                pass
        finally:
            self.sock_conns.discard(sock_conn)

    def setup_websocket(self):
        """Create the websocket server"""
        print("WebSocket server running at ws://localhost:5003")
        self.web_loop= asyncio.new_event_loop()
        asyncio.set_event_loop(self.web_loop)

        socket = websockets.serve(self.sock_conn_handler, "localhost", 5003)

        self.web_loop.run_until_complete(socket)
        self.web_loop.run_forever()
        self.web_loop.close()

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

    async def broadcast_flash_event(self):
        """To every connected socket (should normally only be 1) send the flash event"""
        for conn in self.sock_conns.copy():
            try:
                await conn.send("flash")
            except:
                self.sock_conns.remove(conn)

    def process_usr(self, payload: float) -> None:
        """USR data as input, if beyond threshold then ping display"""
        if payload < 0.2:
            self.web_loop.call_soon_threadsafe(asyncio.create_task, self.broadcast_flash_event())

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
        client.subscribe(self.topics.usr.topic)
        print(f"Subscribed to {self.topics.usr.topic}")

    def on_message(self, _client, _userdata, msg:MQTTMessage):
        '''Callback when receiving a message'''
        payload = json.loads(msg.payload.decode())  # Converts JSON to dict
        # Datermine whether the message is from a room or from a user
        if msg.topic == self.topics.pairing.topic:
            return self.process_pairing(payload)
        if msg.topic == self.topics.usr.topic:
            return self.process_usr(payload)
        else:
            print(f"Unexpected topic: {msg.topic}")
