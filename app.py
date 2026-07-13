from flask import Flask, request
import serial

app = Flask(__name__)
print("APP STARTED")

ser = serial.Serial("/dev/serial0", 115200, timeout=1)

@app.route("/motor/on")
def motor_on():
    print("SEND 1")
    ser.write(b'1')
    return "ON"

@app.route("/motor/off")
def motor_off():
    print("SEND 0")
    ser.write(b'0')
    return "OFF"

@app.route("/dir")
def direction():

    value = request.args.get("value")

    print(f"Direction = {value}")

    if value == "l":
        ser.write(b'l')

    elif value == "r":
        ser.write(b'r')
        
    elif value == "s":
        print("send s")
        ser.write(b's')

    return f"dir = {value}"


# ======================
# YOLO 전용 추가
# ======================

@app.route("/track")
def track():

    value = request.args.get("value")

    print(f"TRACK = {value}")

    if value == "a":
        ser.write(b'a')     # 좌회전

    elif value == "d":
        ser.write(b'd')     # 우회전

    elif value == "f":
        ser.write(b'f')     # 전진

    elif value == "j":
        ser.write(b'j')     # 집기

    return f"track = {value}"




if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
