```python
from flask import Flask, request
import serial
import threading
import logging

app = Flask(__name__)

logging.getLogger("werkzeug").setLevel(logging.ERROR)

ser = serial.Serial("/dev/serial0", 115200, timeout=1)
ser_lock = threading.Lock()


def ser_write(b):
    try:
        with ser_lock:
            ser.write(b)
        return True
    except Exception as e:
        print(f"[SERIAL ERROR] {e}")
        return False


state_lock = threading.Lock()

yolo_enabled = False
saved_direction = "l"
current_mode = "stop"


TRACK_MAP = {
    "L": b'L',
    "R": b'R',
    "a": b'A',
    "d": b'D',
    "f": b'F',
    "b": b'B',
    "e": b'e',
}


@app.route("/motor/on")
def motor_on():
    ser_write(b'1')
    return "ON"


@app.route("/motor/off")
def motor_off():
    ser_write(b'0')
    return "OFF"


@app.route("/dir")
def direction():
    global yolo_enabled, saved_direction, current_mode

    value = request.args.get("value")

    if value == "l":
        with state_lock:
            saved_direction = "l"
        ser_write(b'l')

    elif value == "r":
        with state_lock:
            saved_direction = "r"
        ser_write(b'r')

    elif value == "s":
        with state_lock:
            yolo_enabled = False
            current_mode = "stop"
        ser_write(b's')

    elif value == "y":
        with state_lock:
            yolo_enabled = True
            current_mode = "yolo"
            d = saved_direction
        ser_write(b'y')
        ser_write(b'l' if d == "l" else b'r')

    elif value == "n":
        with state_lock:
            yolo_enabled = False
            current_mode = "normal"
            d = saved_direction
        ser_write(b'n')
        ser_write(b'l' if d == "l" else b'r')

    else:
        return (f"Unknown direction = {value}", 400)

    return f"dir = {value}"


@app.route("/yolo_state")
def yolo_state():
    return "ON" if yolo_enabled else "OFF"


@app.route("/mode")
def mode():
    return current_mode


@app.route("/track")
def track():
    if not yolo_enabled:
        return "YOLO OFF"

    value = request.args.get("value")
    payload = TRACK_MAP.get(value)

    if payload is None:
        return (f"Unknown track = {value}", 400)

    ser_write(payload)
    return f"track = {value}"


@app.route("/manual")
def manual():
    value = request.args.get("value")

    if value not in ("a", "d", "f", "b", "s", "o", "c"):
        return (f"Unknown manual = {value}", 400)

    ser_write(value.encode())
    return f"manual = {value}"


if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5000,
        threaded=True
    )
```
