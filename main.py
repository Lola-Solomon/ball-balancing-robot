import class_BBRobot
import camera
import PID
import time
import threading
import numpy as np

# ===================== CONFIG =====================
ids = [1, 2, 3]

K_PID = [0.105, 0, 0]
k = 0.044117647
a = 2

BALL_AREA_THRESHOLD = 1500
CONTROL_DT = 0.05

# ===================== SHARED VARIABLES =====================
lock = threading.Lock()

image = None
x, y, area = -1, -1, 0

# ===================== STATE =====================
STATE_SEARCH = 0
STATE_TRACK  = 1
state = STATE_SEARCH

# ===================== INIT =====================
Robot = class_BBRobot.BBrobot(ids)
Camera = camera.USBCamera()
pid = PID.pid(K_PID, k, a)

Robot.set_up()
Robot.Initialize_posture()
pz_ini = Robot.ini_pos[2]

goal = [0, 0]

print("✅ System initialized")

# ===================== THREADS (OLD NAMES) =====================
def get_img():
    """Camera acquisition thread"""
    global image
    while True:
        frame = Camera.take_pic()
        if frame is not None:
            with lock:
                image = frame.copy()
        time.sleep(0.002)

def cont_rob():
    """Vision processing thread"""
    global x, y, area
    while True:
        with lock:
            if image is None:
                continue
            img = image.copy()

        bx, by, barea = Camera.find_ball(img)

        with lock:
            x, y, area = bx, by, barea

        time.sleep(0.002)

# ===================== START THREADS =====================
threading.Thread(target=get_img, daemon=True).start()
threading.Thread(target=cont_rob, daemon=True).start()

# ===================== MAIN CONTROL LOOP =====================
try:
    while True:
        with lock:
            bx, by, barea = x, y, area

        # ---------- SEARCH ----------
        if state == STATE_SEARCH:
            pid.integral_x = 0
            pid.integral_y = 0

            if barea > BALL_AREA_THRESHOLD:
                state = STATE_TRACK
                print("🎯 Ball detected → TRACK")

        # ---------- TRACK ----------
        elif state == STATE_TRACK:
            if barea < BALL_AREA_THRESHOLD:
                state = STATE_SEARCH
                pid.integral_x = 0
                pid.integral_y = 0
                print("⚠️ Ball lost → SEARCH")
                time.sleep(CONTROL_DT)
                continue

            # PID compute
            theta, phi = pid.compute(goal, [bx, by])

            # Safety clamp (IMPORTANT)
            phi = max(min(phi, 5.0), -5.0)

            print(
                f"Ball(x={bx:.1f}, y={by:.1f}, area={barea}) | "
                f"Cmd(theta={theta:.2f}, phi={phi:.2f})"
            )

            Robot.control_t_posture([theta, phi, pz_ini], CONTROL_DT)

        time.sleep(CONTROL_DT)

# ===================== CLEAN EXIT =====================
finally:
    print("🛑 Shutting down safely")
    Robot.clean_up()
    Camera.clean_up_cam()
