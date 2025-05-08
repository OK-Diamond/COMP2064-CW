from flask import Flask, render_template, request, jsonify
import qrcode
from io import BytesIO
import base64
import requests
import socket
import os
from flask_cors import CORS

app = Flask(__name__)
CORS(app)  # Enable CORS - Allows cross-origin requests


def get_ip_address():
    """Get the current IP address of the Pi"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


# Consts
PORT = 5002
IP = get_ip_address()
ROBOT_ADDRESS = "?.?.?.?:?"
QR_CODE_URL = f"http://{IP}:{PORT}/register"  # The URL the QR code will point to
ROBOT_API_ENDPOINT = f"http://{ROBOT_ADDRESS}/user"  # Where to send user data


def generate_qr_code():
    """Generate a QR code image and return as base64 string"""
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
    """Main page displaying the QR code"""
    qr_code_image = generate_qr_code()
    return render_template("index.html", qr_code_image=qr_code_image)


@app.route("/display")
def display():
    """Page to be displayed on the Pi's connected screen"""
    qr_code_image = generate_qr_code()
    return render_template("display.html", qr_code_image=qr_code_image)


@app.route("/register")
def register():
    """Form for patients to enter their information"""
    page = "register"
    return render_template(f"{page}.html", page=page)


@app.route("/staff-login")
def staff_login():
    """Login page for staff"""
    page = "staff-login"
    return render_template(f"{page}.html", page=page)

@app.route("/submit", methods=["POST"])
def submit():
    """Endpoint to receive patient information and forward to robot"""
    # Populate user_data with the form data
    user_data = {}
    for i in request.form.keys():
        user_data[i] = request.form.get(i)
    print(jsonify(request.form))
    # Forward data to the robot
    try:
        response = requests.post(ROBOT_API_ENDPOINT, json=user_data)
        if response.status_code == 200:
            return render_template("success.html")
        else:
            return render_template("error.html", error="Robot system unavailable")
    except requests.RequestException:
        return render_template("error.html", error="Cannot connect to robot system")


@app.route("/generate-qr", methods=["GET"])
def generate_qr_api():
    """API endpoint to get QR code image data"""
    qr_code_image = generate_qr_code()
    return jsonify({"qr_code": qr_code_image})


if __name__ == "__main__":
    print(f"Server running at http://{IP}:{PORT}/")

    # Start Flask app
    app.run(host="0.0.0.0", port=PORT, debug=True)
