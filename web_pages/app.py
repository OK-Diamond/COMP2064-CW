'''
Main app
'''

from io import BytesIO
import base64
import json
import threading
import socket
import requests
from paho.mqtt.client import Client as MQTTClient, MQTTMessage
from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
import qrcode
import qrcode.constants


def get_ip_address() -> str:
    '''Get the current IP address of the Pi'''
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except OSError:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


# Global consts
PORT = 5002
IP = get_ip_address()
ROBOT_ADDRESS = "?.?.?.?:?"
QR_CODE_URL = f"http://{IP}:{PORT}/register"  # The URL the QR code will point to
ROBOT_API_ENDPOINT = f"http://{ROBOT_ADDRESS}/user"  # Where to send user data


app = Flask(__name__)
CORS(app)  # Enable CORS - Allows cross-origin requests


class MqttManager:
    '''Manages MQTT requests for IoT'''
    def __init__(self) -> None:
        self.BROKER = "localhost"
        self.PORT = 1883
        self.TOPIC = "hospital/gp/available"  # Topic to subscribe to
        self.patient_queue = []
        self.client = None
        self.queue_lock = threading.Lock()  # For thread safety

    def process_gp_available(self, payload: dict) -> None:
        '''Process a GP availability message'''
        # Example message format: {"room": "3", "ready": true}
        if "room" in payload and payload.get("ready", False):
            room_number = payload["room"]

            with self.queue_lock:
                if self.patient_queue:
                    # Get the next patient from the queue
                    next_patient = self.patient_queue.pop()
                    print(
                        f"GP available in room {room_number}. Next patient: {next_patient['name']}"
                    )
      
                    if self.client:
                        notification = {
                            "action": "collect_patient",
                            "patient_name": next_patient["name"],
                            "room": room_number,
                        }
                        self.client.publish(
                            "hospital/robot/task", json.dumps(notification)
                        )
                else:
                    print(
                        f"GP available in room {room_number}, but no patients in queue"
                    )

    def setup_mqtt(self) -> None:
        '''Setup and connect to MQTT broker'''
        # Callback when connecting
        def on_connect(client, userdata, flags, rc:int) -> None:
            print(f"Connected to MQTT broker with result code {rc}")
            client.subscribe(self.TOPIC)
            print(f"Subscribed to {self.TOPIC}")

        # Callback when receiving a message
        def on_message(client, userdata, msg:MQTTMessage) -> None:
            try:
                payload = json.loads(msg.payload.decode())  # Converts JSON to dict
                self.process_gp_available(payload)
            except json.JSONDecodeError:
                print(f"Error decoding MQTT message: {msg.payload}")

        # Setup client
        client = MQTTClient()
        client.on_connect = on_connect
        client.on_message = on_message

        # Connect to broker
        client.connect(self.BROKER, self.PORT, 60)
        client.loop_start()  # Start background thread for MQTT
        self.client = client
        print("MQTT client started")


def generate_qr_code():
    '''Generate a QR code image and return as base64 string'''
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    qr.add_data(QR_CODE_URL)
    qr.make(fit=True)

    img = qr.make_image(fill_color="black", back_color="white")
    buffered = BytesIO()
    img.save(buffered)
    img_str = base64.b64encode(buffered.getvalue()).decode()
    return img_str


@app.route("/")
def index():
    '''Main page displaying the QR code'''
    qr_code_image = generate_qr_code()
    return render_template("index.html", qr_code_image=qr_code_image)


@app.route("/display")
def display():
    '''Page to be displayed on the Pi's connected screen'''
    qr_code_image = generate_qr_code()
    return render_template("display.html", qr_code_image=qr_code_image)


@app.route("/register")
def register():
    '''Form for patients to enter their information'''
    page = "register"
    return render_template(f"{page}.html", page=page)


@app.route("/staff-login")
def staff_login():
    '''Login page for staff'''
    page = "staff-login"
    return render_template(f"{page}.html", page=page)


@app.route("/submit", methods=["POST"])
def submit():
    '''Endpoint to receive patient information and forward to robot'''
    # Populate user_data with the form data
    user_data = {}
    for i in request.form.keys():
        user_data[i] = request.form.get(i)
    print(jsonify(request.form))
    # Forward data to the robot
    try:
        response = requests.post(ROBOT_API_ENDPOINT, json=user_data, timeout=10)
        if response.status_code == 200:
            return render_template("success.html")
        return render_template("error.html", error="Robot system unavailable")
    except requests.RequestException:
        return render_template("error.html", error="Cannot connect to robot system")


@app.route("/generate-qr", methods=["GET"])
def generate_qr_api():
    '''API endpoint to get QR code image data'''
    qr_code_image = generate_qr_code()
    return jsonify({"qr_code": qr_code_image})


if __name__ == "__main__":
    print(f"Server running at http://{IP}:{PORT}/")

    # Start Flask app
    app.run(host="0.0.0.0", port=PORT, debug=True)
