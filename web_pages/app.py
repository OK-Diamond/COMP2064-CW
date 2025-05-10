'''
Main app
'''

from io import BytesIO
import base64
from queue import Queue, Empty
import time
from socket import socket, AF_INET, SOCK_DGRAM
from flask import Flask, render_template as goto_page, request, jsonify
from flask_cors import CORS
import qrcode
import qrcode.constants
from mqtt import MqttManager
from multithread_datatypes import ThreadsafeRoomList as RoomList
import common
from common import User, MqttTopic, Topics

def get_ip_address() -> str:
    '''Get the current IP address of the Pi'''
    s = socket(AF_INET, SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


# Global consts
PORT = 5002
IP = get_ip_address()
QR_CODE_URL = f"http://{IP}:{PORT}/register"  # The URL the QR code will point to

app = Flask(__name__)
CORS(app)  # Enable CORS - Allows cross-origin requests

TOPICS = Topics(
    MqttTopic("staff","hospital/gp/available"), # Alerts of GP avaliability
    MqttTopic("user","hospital/patient/register"), # Alerts of user registration
    MqttTopic("robot","hospital/robot/task") # Posts here to send instructions to the robot
)
user_queue = Queue()
room_list = RoomList()
mqtt = MqttManager(user_queue, room_list, TOPICS)


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
    #return goto_page("index.html", qr_code_image=generate_qr_code())
    display()


@app.route("/display")
def display():
    '''Page to be displayed on the Pi's connected screen'''
    return goto_page("display.html", qr_code_image=generate_qr_code())


@app.route("/register")
def register():
    '''Form for users to enter their information'''
    page = "register"
    return goto_page(f"{page}.html", page=page)


@app.route("/staff-login")
def staff_login():
    '''Login page for staff'''
    page = "staff-login"
    return goto_page(f"{page}.html", page=page)


@app.route("/submit-register", methods=["POST"])
def submit_register():
    '''Endpoint to receive user information and add to queue'''
    # Create user object
    user = User(request.form.get("name"), request.form.get("dob"), time.time())

    # Add to the queue
    mqtt.post_user(user)
    queue_position = user_queue.qsize()

    print(f"Added user to queue: {user.name}, position estimate: {queue_position}")
    return goto_page("success.html")


@app.route("/submit-staff-login", methods=["POST"])
def submit_staff_login():
    '''Endpoint for staff login'''
    staff_id = request.form.get("name")
    room:int = request.form.get("room")

    # Add room to avaliable rooms list
    room_list.add_room(room)

    print(f"Staff login attempt: {staff_id}")
    return goto_page("staff-dashboard.html", queue=user_queue)


@app.route("/generate-qr", methods=["GET"])
def generate_qr_api():
    '''API endpoint to get QR code image data'''
    qr_code_image = generate_qr_code()
    return jsonify({"qr_code": qr_code_image})


if __name__ == "__main__":
    # Start Flask app
    print(f"Server running at http://{IP}:{PORT}/")
    app.run(host="0.0.0.0", port=PORT, debug=True)
