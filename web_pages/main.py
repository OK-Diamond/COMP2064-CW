'''
Main app
'''

from io import BytesIO
import base64
from queue import Queue
import time
from socket import socket, AF_INET, SOCK_DGRAM
from flask import Flask, render_template as goto_page, request, jsonify
from flask_cors import CORS
import qrcode
import qrcode.constants
from mqtt import MqttManager
from multithread_datatypes import ThreadsafeRoomList as RoomList
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


class FlaskServer:
    '''Manages the Flask server and routes'''
    PORT = 5002
    IP = get_ip_address()
    QR_CODE_URL = f"http://{IP}:{PORT}/register"  # The URL the QR code will point to

    def __init__(self, mqtt_manager:MqttManager) -> None:
        self.mqtt = mqtt_manager
        # Setup Flask app
        self.app = Flask(__name__)
        CORS(self.app)  # Enable CORS - Allows cross-origin requests
        self.setup_routes()

    def run(self) -> None:
        '''Run the Flask app'''
        self.app.run(host="0.0.0.0", port=self.PORT)
        print(f"Server running at http://{self.IP}:{self.PORT}/")

    def generate_qr_code(self) -> str:
        '''Generate a QR code image and return as base64 string'''
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=10,
            border=4,
        )
        qr.add_data(self.QR_CODE_URL)
        qr.make(fit=True)

        img = qr.make_image(fill_color="black", back_color="white")
        buffered = BytesIO()
        img.save(buffered)
        img_str = base64.b64encode(buffered.getvalue()).decode()
        return img_str

    def setup_routes(self) -> None:
        '''Sets up the routes for the web app.'''
        self.app.route("/")(self.index)
        self.app.route("/qr")(self.qr)
        self.app.route("/display")(self.display)
        self.app.route("/register")(self.register)
        self.app.route("/staff-login")(self.staff_login)
        self.app.route("/submit-register", methods=["POST"])(self.submit_register)
        self.app.route("/submit-staff-login", methods=["POST"])(self.submit_staff_login)
        self.app.route("/generate-qr", methods=["GET"])(self.generate_qr_api)

    def page_template(self, page:str) -> str:
        '''Helper function to render a page with no args'''
        return goto_page(f"{page}.html", page=page)

    def index(self) -> str:
        '''Main page'''
        return self.page_template("index")

    def qr(self) -> str:
        '''Page displaying the QR code'''
        return goto_page("qr.html", qr_code_image=self.generate_qr_code())

    def display(self) -> str:
        '''Waiting room display'''
        return self.page_template("display")

    def register(self) -> str:
        '''Form for users to enter their information'''
        return self.page_template("register")

    def staff_login(self) -> str:
        '''Login page for staff'''
        return self.page_template("staff-login")

    def submit_register(self) -> str:
        '''Endpoint to receive user information and add to queue'''
        # Create user object
        user = User(request.form.get("name"), request.form.get("dob"), time.time())
        # Add to the queue
        self.mqtt.post_user(user)
        queue_position = self.mqtt.user_queue.qsize()
        print(f"Added user to queue: {user.name}, position estimate: {queue_position}")
        return goto_page("success.html")

    def submit_staff_login(self) -> str:
        '''Endpoint for staff login'''
        staff_id = request.form.get("name")
        room:int = request.form.get("room")
        # Add room to avaliable rooms list
        self.mqtt.post_gp_available(room, True)
        print(f"Staff login attempt: {staff_id}")
        return goto_page("staff-dashboard.html")

    def generate_qr_api(self) -> str:
        '''API endpoint to get QR code image data'''
        qr_code_image = self.generate_qr_code()
        return jsonify({"qr_code": qr_code_image})


if __name__ == "__main__":
    TOPICS = Topics(
        MqttTopic("staff","hospital/gp/available"), # Alerts of GP avaliability
        MqttTopic("user","hospital/patient/register"), # Alerts of user registration
        MqttTopic("pairing","hospital/pairing") # Alerts of gp-patient pairing
    )
    mqtt = MqttManager(Queue(), RoomList(), TOPICS)
    # Start Flask app
    server = FlaskServer(mqtt)
    server.run()
