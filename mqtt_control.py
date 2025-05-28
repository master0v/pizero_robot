#!/usr/bin/env python3
import sys
import time
import json
import base64

import cv2
import numpy as np
import paho.mqtt.client as mqtt

from picamera2 import Picamera2
from pupil_apriltags import Detector
from vedo import load, Box, Arrow, Assembly, Plotter, Text3D

# ─── your reusable modules ────────────────────────────────────────────────────
import ledControl
import move
import ultra

# ─── for servo (inline, no curses) ─────────────────────────────────────────────
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as servomodule

# ─── MQTT topics ──────────────────────────────────────────────────────────────
BROKER                   = "192.168.0.31"
PORT                     = 1883

TOPIC_IMG_REQ            = "robot/image/request"
TOPIC_IMG                = "robot/image"

TOPIC_LED                = "robot/led"
TOPIC_MOVE               = "robot/move"
TOPIC_DIST_REQ           = "robot/distance/request"
TOPIC_DIST               = "robot/distance"
TOPIC_SERVO              = "robot/servo"

# ─── CAMERA SETUP (manual exposure still‐mode) ─────────────────────────────────
RES           = (640, 480)
JPEG_QUAL     = 40

picam2 = Picamera2()
still_conf = picam2.create_still_configuration(
    main={"size": RES, "format": "RGB888"}
)
picam2.configure(still_conf)

# warm up auto‐exposure / white‐balance
picam2.start()
time.sleep(2.0)
# grab one metadata snapshot to lock exposure/gain
req = picam2.capture_request()
md  = req.get_metadata()
exposure_time = int(md.get("ExposureTime", 0))
analogue_gain = md.get("AnalogueGain", 1.0)
req.release()
picam2.stop()

picam2.set_controls({
    "AwbEnable": False,
    "ExposureTime": exposure_time,
    "AnalogueGain": analogue_gain
})
print(f"[CAMERA] locked EX={exposure_time}µs, AG={analogue_gain:.2f}")

# ─── LED / MOTOR / ULTRASONIC / SERVO SETUP ───────────────────────────────────
lc = ledControl.ledControl()

move.setup()           # initialize GPIO & PWM for motors
ultra.init_sensor()    # init HC-SR04

# servo/channel from test_servo.py
i2c     = busio.I2C(board.SCL, board.SDA)
pca     = PCA9685(i2c)
pca.frequency = 50
my_servo = servomodule.Servo(
    pca.channels[11],
    min_pulse=500, max_pulse=2500
)
SERVO_MIN_ANGLE = 30
SERVO_MAX_ANGLE = 90
print(f"[SERVO] ready on channel 11 ({SERVO_MIN_ANGLE}–{SERVO_MAX_ANGLE}°)")

# ─── APRILTAG & VEDO SETUP ─────────────────────────────────────────────────────
scene = load("2305aug_zero_in_my_corner.obj")
with open("2305_tags.json") as f:
    tag_data = json.load(f)

actors = [scene]
for tag in tag_data:
    size = tag.get("size", 0.1)
    T = np.array(tag["transform"], dtype=float)
    if np.max(np.abs(T[:3,3])) > 10.0:
        T[:3,3] /= 1000.0

    plate = Box(pos=(0,0,0), length=size, width=size, height=0.01).c("blue").alpha(1.0)
    plate.apply_transform(T)
    actors.append(plate)

    arrow = Arrow((0,0,0),(0,0,size)).c("red").lw(4).alpha(1.0)
    arrow.apply_transform(T)
    actors.append(arrow)

plt = Plotter(axes=1,
              title="AprilTag + Camera Pose",
              size=(1200, 800))
plt.show(actors, interactive=False)
camera_actor = None

intr = np.load("camera_intrinsics.npz")
K, D = intr["K"], intr["D"]
detector = Detector(families="tag36h11")

def pose_from_rvec_tvec(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=float)
    T[:3,:3] = R
    T[:3, 3] = tvec.flatten()
    return T

def publish_image():
    # start pipeline briefly to capture
    picam2.start()
    time.sleep(0.05)
    frame = picam2.capture_array("main")
    picam2.stop()

    # encode & publish
    ret, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUAL])
    if not ret:
        print("[IMAGE] JPEG encode failed")
        return
    jpg = buf.tobytes()
    payload = json.dumps({
        "timestamp": time.time(),
        "image_b64": base64.b64encode(jpg).decode('ascii')
    })
    client.publish(TOPIC_IMG, payload)
    print(f"[IMAGE] published ({len(jpg)} bytes)")

def handle_image_request():
    publish_image()

def handle_led(msg):
    """
    payload: {"cmd":"red","wait_ms":100}
     cmd ∈ ["red","green","blue","wipeClean","rainbow","rainbowCycle","theaterChase","theaterChaseRainbow"]
    """
    try:
        p = json.loads(msg.payload.decode())
        cmd = p.get("cmd","").lower()
        wait = p.get("wait_ms",50)
        if cmd == "red":
            lc.redColorWipe(wait)
        elif cmd == "green":
            lc.greenColorWipe(wait)
        elif cmd == "blue":
            lc.blueColorWipe(wait)
        elif cmd == "wipeclean":
            lc.wipeClean()
        elif cmd == "rainbow":
            lc.rainbow(lc.strip, wait, iterations=1)
        elif cmd == "rainbowcycle":
            lc.rainbowCycle(lc.strip, wait, iterations=1)
        elif cmd == "theaterchase":
            lc.theaterChase(lc.strip, p.get("color", lc.RED), wait, iterations=5)
        elif cmd == "theaterchaserainbow":
            lc.theaterChaseRainbow(lc.strip, wait)
        else:
            print(f"[LED] unknown cmd '{cmd}'")
            return
        print(f"[LED] ran '{cmd}'")
    except Exception as e:
        print(f"[LED] error: {e}")

def handle_move(msg):
    """
    payload: {"left":50,"right":-30}
    """
    try:
        p = json.loads(msg.payload.decode())
        left  = int(p.get("left",0))
        right = int(p.get("right",0))
        move.drive(left, right)
        print(f"[MOVE] L={left}%, R={right}%")
    except Exception as e:
        print(f"[MOVE] error: {e}")

def handle_distance_request():
    dist = ultra.get_distance()
    out = {"timestamp": time.time(), "distance_m": dist if dist is not None else None}
    client.publish(TOPIC_DIST, json.dumps(out))
    print(f"[DIST] {out}")

def handle_servo(msg):
    """
    payload: {"angle":45}
    """
    try:
        p = json.loads(msg.payload.decode())
        a = float(p.get("angle", SERVO_MIN_ANGLE))
        a = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, a))
        my_servo.angle = a
        print(f"[SERVO] set to {a:.1f}°")
    except Exception as e:
        print(f"[SERVO] error: {e}")

# ─── MQTT SETUP ────────────────────────────────────────────────────────────────
client = mqtt.Client()

def on_connect(c, userdata, flags, rc):
    print(f"[MQTT] connected (rc={rc})")
    for t in (TOPIC_IMG_REQ,
              TOPIC_LED,
              TOPIC_MOVE,
              TOPIC_DIST_REQ,
              TOPIC_SERVO):
        c.subscribe(t)
        print(f"[MQTT] sub {t!r}")

def on_message(c, userdata, msg):
    topic = msg.topic
    if topic == TOPIC_IMG_REQ:
        handle_image_request()
    elif topic == TOPIC_LED:
        handle_led(msg)
    elif topic == TOPIC_MOVE:
        handle_move(msg)
    elif topic == TOPIC_DIST_REQ:
        handle_distance_request()
    elif topic == TOPIC_SERVO:
        handle_servo(msg)
    else:
        print(f"[MQTT] unhandled topic {topic!r}")

client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, keepalive=60)
client.loop_start()
print("[MQTT] loop started, idle…")

try:
    while True:
        # keep vedo alive
        plt.render()
        time.sleep(0.1)
except KeyboardInterrupt:
    print("[MAIN] exit requested")
finally:
    client.loop_stop()
    client.disconnect()
    move.destroy()
    pca.deinit()
    ultra.GPIO.cleanup()
    print("[MAIN] clean exit")
    sys.exit(0)
