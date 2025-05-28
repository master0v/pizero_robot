#!/usr/bin/env python3
import time
import json
import base64
import cv2
from picamera2 import Picamera2
import paho.mqtt.client as mqtt

# ─── CONFIG ───────────────────────────────────────────────────────────────────
BROKER    = "192.168.0.31"     # your MQTT broker
PORT      = 1883
TOPIC_IMG = "robot/image"      # topic where images are published
FPS       = 1                  # frames per second
RES       = (640, 480)         # lower resolution to save bandwidth & CPU
JPEG_QUAL = 30                 # JPEG quality (0–100)

# ─── SETUP MQTT ───────────────────────────────────────────────────────────────
mqtt_client = mqtt.Client()
mqtt_client.connect(BROKER, PORT)
mqtt_client.loop_start()

# ─── SETUP CAMERA ─────────────────────────────────────────────────────────────
picam2 = Picamera2()
video_conf = picam2.create_video_configuration(
    main={"size": RES, "format": "BGR888"}
)
picam2.configure(video_conf)
picam2.start()
time.sleep(2)  # allow sensor to warm up

print(f"Streaming {FPS} FPS @ {RES[0]}×{RES[1]} to {BROKER}:{PORT}/{TOPIC_IMG}")
try:
    interval = 1.0 / FPS
    while True:
        t0 = time.time()
        ts = time.time()

        # capture a frame (XRGB → BGR)
        frame = picam2.capture_array("main")
        bgr   = frame[:, :, 2::-1]

        # JPEG-encode
        ret, buf = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUAL])
        jpg_bytes = buf.tobytes()
        jpg_b64   = base64.b64encode(jpg_bytes).decode('ascii')

        # build JSON payload
        payload = json.dumps({
            "timestamp": ts,
            "image_b64": jpg_b64
        })

        # publish
        mqtt_client.publish(TOPIC_IMG, payload)
        print(f"[{ts:.3f}] sent {len(jpg_bytes)}-byte JPEG")

        # maintain target FPS
        dt = time.time() - t0
        if dt < interval:
            time.sleep(interval - dt)

except KeyboardInterrupt:
    print("\nStopping stream…")

finally:
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    picam2.stop()
