from ultralytics import YOLO
import cv2
import requests
import time

SERVER = "http://127.0.0.1:5000"

CENTER_TOLERANCE = 50
PICKUP_Y = 450
FINAL_APPROACH_TIME = 2.0

model = YOLO("/home/pi/yolo-Weights/best.pt")
classNames = model.names

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

prev_state = -1

pickup_mode = False
pickup_start = 0

PICKUP_COOLDOWN = 3.0
last_pickup_time = 0


# =========================
# 서버 통신 함수
# =========================

def track_left():
    try:
        requests.get(
            f"{SERVER}/track",
            params={"value": "a"},
            timeout=0.2
        )
    except:
        pass


def track_right():
    try:
        requests.get(
            f"{SERVER}/track",
            params={"value": "d"},
            timeout=0.2
        )
    except:
        pass


def track_forward():
    try:
        requests.get(
            f"{SERVER}/track",
            params={"value": "f"},
            timeout=0.2
        )
    except:
        pass


def pickup():
    try:
        requests.get(
            f"{SERVER}/track",
            params={"value": "p"},
            timeout=0.2
        )
    except:
        pass


# =========================

while True:
    ret, img = cap.read()

    if not ret:
        break

    img = cv2.flip(img, 1)

    if time.time() - last_pickup_time < PICKUP_COOLDOWN:
        remain = PICKUP_COOLDOWN - (
            time.time() - last_pickup_time
        )

        cv2.putText(
            img,
            f"WAIT {remain:.1f}s",
            (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 255),
            3
        )

        cv2.imshow("Robot Tracking", img)

        if cv2.waitKey(1) & 0xFF == 27:
            break

        continue

    h, w = img.shape[:2]

    frame_cx = w // 2
    frame_cy = h // 2

    state = 0
    state_text = "NO TARGET"

    results = model(img, verbose=False)[0]

    cv2.line(img, (frame_cx, 0), (frame_cx, h), (255, 0, 0), 1)
    cv2.line(img, (0, frame_cy), (w, frame_cy), (255, 0, 0), 1)

    cv2.line(
        img,
        (0, PICKUP_Y),
        (w, PICKUP_Y),
        (0, 0, 255),
        2
    )

    best_box = None
    best_conf = 0
    best_name = None

    for box in results.boxes:
        cls = int(box.cls[0])
        conf = float(box.conf[0]) * 100
        name = classNames[cls]

        if name not in ["PlasticBottle", "can"]:
            continue

        if conf < 40:
            continue

        if conf > best_conf:
            best_conf = conf
            best_box = box
            best_name = name

    # =========================
    # 객체 발견
    # =========================

    if best_box is not None:
        x1, y1, x2, y2 = map(int, best_box.xyxy[0])

        obj_cx = (x1 + x2) // 2
        obj_cy = (y1 + y2) // 2

        offset_x = obj_cx - frame_cx

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
            5,
            (0, 0, 255),
            -1
        )

        cv2.line(
            img,
            (frame_cx, frame_cy),
            (obj_cx, obj_cy),
            (0, 255, 255),
            2
        )

        # =========================
        # 마지막 접근
        # =========================

        if pickup_mode:
            elapsed = time.time() - pickup_start

            if elapsed < FINAL_APPROACH_TIME:
                state = 3
                state_text = (
                    f"FINAL APPROACH "
                    f"{elapsed:.1f}s"
                )
            else:
                state = 4
                state_text = "PICKUP"
        else:
            if obj_cy >= PICKUP_Y:
                pickup_mode = True
                pickup_start = time.time()

                state = 3
                state_text = "FINAL APPROACH"
            else:
                if offset_x > CENTER_TOLERANCE:
                    state = 1
                    state_text = "TURN LEFT"
                elif offset_x < -CENTER_TOLERANCE:
                    state = 2
                    state_text = "TURN RIGHT"
                else:
                    state = 3
                    state_text = "FORWARD"

        cv2.putText(
            img,
            f"{best_name} {best_conf:.1f}%",
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.putText(
            img,
            f"X:{offset_x}",
            (x1, y2 + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2
        )

        cv2.putText(
            img,
            f"Y:{obj_cy}",
            (x1, y2 + 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2
        )

    else:
        state = 0
        state_text = "NO TARGET"

    # =========================
    # 상태 변경 시에만 전송
    # =========================

    if state != prev_state:
        print(
            f"State Changed -> "
            f"{state} ({state_text})"
        )

        if state == 0:
            track_forward()
        elif state == 1:
            track_left()
        elif state == 2:
            track_right()
        elif state == 3:
            track_forward()
        elif state == 4:
            pickup()

            print("PICKUP COMPLETE")

            last_pickup_time = time.time()

            pickup_mode = False
            pickup_start = 0

            prev_state = -1
            state = 0

        prev_state = state

    # =========================
    # PICKUP 후 종료
    # =========================

    cv2.rectangle(
        img,
        (10, 10),
        (500, 70),
        (0, 0, 0),
        -1
    )

    cv2.putText(
        img,
        f"CAR : {state_text}",
        (20, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        3
    )

    cv2.imshow(
        "Robot Tracking",
        img
    )

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
