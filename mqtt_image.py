#!/usr/bin/env python3
import time
import json
import base64
import cv2
from picamera2 import Picamera2
import paho.mqtt.client as mqtt

# ─── CONFIG ────────────────────────────────────────────────────────────────────
BROKER        = "192.168.0.31"
PORT          = 1883
TOPIC_REQUEST = "robot/image/request"
TOPIC_IMAGE   = "robot/image"
RES           = (640, 480)
JPEG_QUAL     = 40

# ─── SETUP CAMERA ──────────────────────────────────────────────────────────────
picam2 = Picamera2()

# 1) Prepare a still‐capture pipeline
still_conf = picam2.create_still_configuration(
    main={"size": RES, "format": "RGB888"}
)
picam2.configure(still_conf)

# 2) Start once, warm up auto‐exposure & white‐balance
picam2.start()
time.sleep(2.0)

# 3) Grab one request to read the chosen AE/WB settings
req = picam2.capture_request()
md  = req.get_metadata()
# these keys may vary by firmware—inspect `md` if these fail:
exposure_time = int(md.get("ExposureTime", 0))
analogue_gain = md.get("AnalogueGain", 1.0)
req.release()

# 4) Stop the pipeline and switch to manual controls
picam2.stop()
picam2.set_controls({
    "AwbEnable": False,
    "ExposureTime": exposure_time,
    "AnalogueGain": analogue_gain
})
print(f"[INFO] Locked EX={exposure_time}µs, AG={analogue_gain:.2f}")

# ─── MQTT CALLBACKS ───────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc):
    client.subscribe(TOPIC_REQUEST)
    print(f"[INFO] MQTT connected; subscribed to '{TOPIC_REQUEST}'")

def on_message(client, userdata, msg):
    if msg.topic != TOPIC_REQUEST:
        return

    # 1) start pipeline just for this shot
    picam2.start()
    # a very short pause to let the controls take effect
    time.sleep(0.05)

    # 2) capture
    frame = picam2.capture_array("main")

    # 3) stop immediately
    picam2.stop()

    # 4) JPEG encode
    ret, buf = cv2.imencode('.jpg', frame,
                            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUAL])
    if not ret:
        print("[WARN] JPEG encode failed")
        return

    # 5) package & publish
    jpg_bytes = buf.tobytes()
    payload = json.dumps({
        "timestamp": time.time(),
        "image_b64": base64.b64encode(jpg_bytes).decode('ascii')
    })
    client.publish(TOPIC_IMAGE, payload)
    print(f"[{time.time():.3f}] Published frame ({len(jpg_bytes)} bytes)")

# ─── MAIN ─────────────────────────────────────────────────────────────────────
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

mqtt_client.connect(BROKER, PORT, keepalive=60)
mqtt_client.loop_start()
print("[INFO] MQTT loop running; idle until requests come in…")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("[INFO] Shutting down…")
finally:
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    print("[INFO] Clean exit")
