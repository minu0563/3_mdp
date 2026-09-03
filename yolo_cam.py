from ultralytics import YOLO
import cv2
import requests
import time
import threading
from collections import deque

SERVER = "http://127.0.0.1:5000"

SHOW_WINDOW = True

PICKUP_Y = 300
BACKUP_Y = 410

CENTER_TOLERANCE = 60
PICKUP_TOLERANCE = 45

COARSE_TURN_THRESHOLD = 130

EAT_CONFIRM_FRAMES = 3

PICKUP_COOLDOWN = 3.0

CONF_THRESHOLD = 40
TARGET_CLASSES = ("PlasticBottle", "can")

HEARTBEAT_INTERVAL = 0.08

model = YOLO("/home/pi/yolo-Weights/best_ncnn_model")
classNames = model.names


class CameraStream:

    def __init__(self, src=0, width=640, height=480):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.lock = threading.Lock()
        self.ret, self.frame = self.cap.read()
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret, self.frame = ret, frame

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ret, self.frame.copy()

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()


class CommandSender:

    def __init__(self, server, interval=HEARTBEAT_INTERVAL):
        self.server = server
        self.interval = interval
        self.session = requests.Session()
        self.lock = threading.Lock()
        self.pending = None
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def send(self, value):
        with self.lock:
            self.pending = value

    def send_now(self, value):
        try:
            self.session.get(
                f"{self.server}/track",
                params={"value": value},
                timeout=0.5
            )
        except Exception:
            pass

    def _worker(self):
        last_sent = 0.0

        while self.running:
            cmd = None

            if time.time() - last_sent >= self.interval:
                with self.lock:
                    cmd = self.pending
                    self.pending = None

            if cmd is None:
                time.sleep(0.01)
                continue

            try:
                self.session.get(
                    f"{self.server}/track",
                    params={"value": cmd},
                    timeout=0.3
                )
            except Exception:
                pass

            last_sent = time.time()

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)


class StateWatcher:

    def __init__(self, server, interval=0.2):
        self.server = server
        self.interval = interval
        self.session = requests.Session()
        self.enabled = False
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        while self.running:
            try:
                r = self.session.get(
                    f"{self.server}/yolo_state",
                    timeout=0.3
                )
                self.enabled = (r.text.strip() == "ON")
            except Exception:
                self.enabled = False

            time.sleep(self.interval)

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)


cam = CameraStream(0, 640, 480)
sender = CommandSender(SERVER)
watcher = StateWatcher(SERVER)

last_pickup_time = 0.0
eat_confirm_count = 0

offset_history = deque(maxlen=5)
cy_history = deque(maxlen=5)


def median_of(hist, value):
    hist.append(value)
    s = sorted(hist)
    return s[len(s) // 2]


def reset_tracking():
    global eat_confirm_count

    eat_confirm_count = 0
    offset_history.clear()
    cy_history.clear()


try:
    while True:
        ret, img = cam.read()

        if not ret or img is None:
            time.sleep(0.005)
            continue

        img = cv2.flip(img, 1)

        if not watcher.enabled:
            reset_tracking()

            if SHOW_WINDOW:
                cv2.rectangle(
                    img,
                    (10, 10),
                    (520, 70),
                    (0, 0, 0),
                    -1
                )

                cv2.putText(
                    img,
                    "YOLO : OFF",
                    (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 255),
                    3
                )

                cv2.imshow("Robot Tracking", img)

                if cv2.waitKey(1) & 0xFF == 27:
                    break

            else:
                time.sleep(0.05)

            continue

        if time.time() - last_pickup_time < PICKUP_COOLDOWN:
            reset_tracking()

            if SHOW_WINDOW:
                remain = PICKUP_COOLDOWN - (
                    time.time() - last_pickup_time
                )

                cv2.rectangle(
                    img,
                    (10, 10),
                    (520, 70),
                    (0, 0, 0),
                    -1
                )

                cv2.putText(
                    img,
                    f"EATING... {remain:.1f}s",
                    (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 255),
                    3
                )

                cv2.imshow("Robot Tracking", img)

                if cv2.waitKey(1) & 0xFF == 27:
                    break

            else:
                time.sleep(0.05)

            continue

        h, w = img.shape[:2]
        frame_cx = w // 2

        cmd = None
        state_text = "NO TARGET"
        state_color = (0, 255, 0)

        results = model(img, verbose=False)[0]

        best_box, best_conf, best_name = None, 0, None

        for box in results.boxes:
            name = classNames[int(box.cls[0])]

            if name not in TARGET_CLASSES:
                continue

            conf = float(box.conf[0]) * 100

            if conf < CONF_THRESHOLD:
                continue

            if conf > best_conf:
                best_conf, best_box, best_name = conf, box, name

        if SHOW_WINDOW:
            cv2.line(
                img,
                (frame_cx, 0),
                (frame_cx, h),
                (255, 0, 0),
                1
            )

            cv2.line(
                img,
                (frame_cx - PICKUP_TOLERANCE, PICKUP_Y),
                (frame_cx - PICKUP_TOLERANCE, h),
                (255, 200, 0),
                1
            )

            cv2.line(
                img,
                (frame_cx + PICKUP_TOLERANCE, PICKUP_Y),
                (frame_cx + PICKUP_TOLERANCE, h),
                (255, 200, 0),
                1
            )

            cv2.line(
                img,
                (0, PICKUP_Y),
                (w, PICKUP_Y),
                (0, 255, 0),
                2
            )

            cv2.line(
                img,
                (0, BACKUP_Y),
                (w, BACKUP_Y),
                (0, 0, 255),
                2
            )

        if best_box is not None:
            x1, y1, x2, y2 = map(
                int,
                best_box.xyxy[0]
            )

            obj_cx = (x1 + x2) // 2
            obj_cy_raw = (y1 + y2) // 2

            offset_x = median_of(
                offset_history,
                obj_cx - frame_cx
            )

            obj_cy = median_of(
                cy_history,
                obj_cy_raw
            )

            if SHOW_WINDOW:
                cv2.rectangle(
                    img,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                cv2.circle(
                    img,
                    (obj_cx, obj_cy),
                    6,
                    (0, 0, 255),
                    -1
                )

            if obj_cy > BACKUP_Y:
                eat_confirm_count = 0
                cmd = "b"
                state_text = f"TOO CLOSE - BACK UP (y={obj_cy})"
                state_color = (0, 165, 255)

            elif obj_cy >= PICKUP_Y:
                if abs(offset_x) <= PICKUP_TOLERANCE:
                    eat_confirm_count += 1

                    if eat_confirm_count >= EAT_CONFIRM_FRAMES:
                        sender.send_now("e")
                        last_pickup_time = time.time()
                        reset_tracking()
                        cmd = None
                        state_text = "*** EAT ***"
                        state_color = (0, 0, 255)

                    else:
                        cmd = None
                        state_text = (
                            f"ALIGNED - CONFIRM "
                            f"{eat_confirm_count}/{EAT_CONFIRM_FRAMES}"
                        )
                        state_color = (0, 255, 255)

                else:
                    eat_confirm_count = 0

                    if abs(offset_x) > COARSE_TURN_THRESHOLD:
                        cmd = "L" if offset_x > 0 else "R"
                    else:
                        cmd = "a" if offset_x > 0 else "d"

                    state_text = (
                        f"PICKUP ZONE - ALIGN ({offset_x:+d})"
                    )
                    state_color = (0, 255, 255)

            else:
                eat_confirm_count = 0

                if offset_x > CENTER_TOLERANCE:
                    if offset_x > COARSE_TURN_THRESHOLD:
                        cmd = "L"
                        state_text = (
                            f"TURN LEFT >> ({offset_x:+d})"
                        )
                    else:
                        cmd = "a"
                        state_text = (
                            f"turn left ({offset_x:+d})"
                        )

                elif offset_x < -CENTER_TOLERANCE:
                    if offset_x < -COARSE_TURN_THRESHOLD:
                        cmd = "R"
                        state_text = (
                            f"TURN RIGHT >> ({offset_x:+d})"
                        )
                    else:
                        cmd = "d"
                        state_text = (
                            f"turn right ({offset_x:+d})"
                        )

                else:
                    cmd = "f"
                    state_text = "FORWARD"

            if SHOW_WINDOW:
                cv2.putText(
                    img,
                    f"{best_name} {best_conf:.0f}%",
                    (x1, max(y1 - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    img,
                    f"X:{offset_x:+d} Y:{obj_cy}",
                    (x1, min(y2 + 25, h - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )

        else:
            reset_tracking()
            cmd = "f"
            state_text = "NO TARGET"
            state_color = (128, 128, 128)

        if cmd is not None:
            sender.send(cmd)

        if SHOW_WINDOW:
            cv2.rectangle(
                img,
                (10, 10),
                (620, 70),
                (0, 0, 0),
                -1
            )

            cv2.putText(
                img,
                state_text,
                (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                state_color,
                2
            )

            cv2.imshow("Robot Tracking", img)

            if cv2.waitKey(1) & 0xFF == 27:
                break

finally:
    try:
        requests.get(
            f"{SERVER}/dir",
            params={"value": "s"},
            timeout=0.5
        )
    except Exception:
        pass

    sender.stop()
    watcher.stop()
    cam.stop()
    cv2.destroyAllWindows()
