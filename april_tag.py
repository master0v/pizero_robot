#!/usr/bin/env python3

# REQUIREMENTS:
# sudo apt install python3-opencv
# pip3 install apriltag

"""
ultra_servo_apriltag.py

Live PiCamera2 → Tkinter video with:
  - ultrasonic distance overlay
  - crosshair
  - servo angle overlay
  - real-time AprilTag detection and ID overlay
  - Up/Down keys to drive a PCA9685-driven servo
"""

import os
# suppress libcamera INFO/WARN (only ERROR+ appear)
os.environ["LIBCAMERA_LOG_LEVELS"]  = "ERROR"
os.environ["LIBCAMERA_LOG_NO_COLOR"] = "1"

import signal
import time
import io

import ultra                # your ultra.py (must define init_sensor & get_distance)
import RPi.GPIO as GPIO
import tkinter as tk
from PIL import Image, ImageDraw, ImageFont
from picamera2 import Picamera2

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

import cv2
import numpy as np
import apriltag

# ────────────────────────────────────────────────────────
# Servo configuration
MIN_ANGLE = 30
MAX_ANGLE = 90
STEP      = 5       # degrees per keypress
CHANNEL   = 11      # PCA9685 channel

# ────────────────────────────────────────────────────────
# Graceful shutdown flag
_stop = False
def _on_sigint(signum, frame):
    global _stop
    _stop = True

signal.signal(signal.SIGINT,  _on_sigint)
signal.signal(signal.SIGTERM, _on_sigint)

def main():
    # 1) init ultrasonic
    ultra.init_sensor()

    # 2) init servo
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50
    my_servo = servo.Servo(
        pca.channels[CHANNEL],
        min_pulse=500, max_pulse=2500
    )
    angle = MAX_ANGLE
    my_servo.angle = angle

    # 3) setup Picamera2 → RGB888 but we’ll swap to real RGB below
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={
        "format": "RGB888",     # actually comes out BGR
        "size":   (640, 480)
    })
    picam2.configure(config)
    picam2.start()

    # 4) setup AprilTag detector
    detector = apriltag.Detector()

    # 5) setup Tkinter
    root = tk.Tk()
    root.title("UltraCam + Servo + AprilTag")
    root.focus_force()
    canvas = tk.Canvas(root, width=640, height=480)
    canvas.pack()

    # preload font
    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24
    )

    def on_key(event):
        nonlocal angle
        key = event.keysym
        if key == 'Up':
            new = max(angle - STEP, MIN_ANGLE)
        elif key == 'Down':
            new = min(angle + STEP, MAX_ANGLE)
        elif key in ('q', 'Q'):
            cleanup()
            root.quit()
            return
        else:
            return

        if new != angle:
            angle = new
            my_servo.angle = angle

    root.bind("<Key>", on_key)

    def cleanup():
        my_servo.angle = None
        pca.deinit()
        picam2.stop()
        picam2.close()
        GPIO.cleanup()

    def update_frame():
        if _stop:
            cleanup()
            root.quit()
            return

        # 1) grab frame as true RGB via swap
        frame = picam2.capture_array("main")        # B, G, R
        frame = frame[..., ::-1]                    # swap → R, G, B
        img = Image.fromarray(frame, mode="RGB")
        draw = ImageDraw.Draw(img)
        w, h = img.size
        cx, cy, L = w // 2, h // 2, 20
        col = (255, 0, 0, 255)

        # 2) crosshair
        draw.line([(cx-L, cy), (cx+L, cy)], fill=col, width=3)
        draw.line([(cx, cy-L), (cx, cy+L)], fill=col, width=3)

        # 3) ultrasonic distance
        dist = ultra.get_distance()
        dist_txt = "Out of range" if dist is None else f"{dist:.2f} m"
        x0, y0, x1, y1 = draw.textbbox((0,0), dist_txt, font=font)
        tw, th = x1-x0, y1-y0
        draw.text(((w-tw)/2, cy+L+10), dist_txt, font=font, fill=col)

        # 4) AprilTag detection
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections = detector.detect(gray)
        for det in detections:
            # corners is 4×2 array:  (top-left, top-right, bottom-right, bottom-left)
            pts = det.corners.astype(int).reshape((4,2))
            # draw polygon
            for i in range(4):
                pt1 = tuple(pts[i])
                pt2 = tuple(pts[(i+1)%4])
                draw.line([pt1, pt2], fill=(0,255,0,255), width=3)
            # draw the tag ID at its center
            cx_tag, cy_tag = int(det.center[0]), int(det.center[1])
            draw.text((cx_tag+5, cy_tag+5), str(det.tag_id),
                      font=font, fill=(0,255,0,255))

        # 5) servo angle text
        angle_txt = f"[↑:↓:Q] Servo: {angle:.1f}°"
        draw.text((10, 10), angle_txt, font=font, fill=col)

        # 6) render to Tk
        buf = io.BytesIO()
        img.save(buf, format="PPM")
        tkimg = tk.PhotoImage(data=buf.getvalue())
        canvas.create_image(0, 0, anchor="nw", image=tkimg)
        canvas.img = tkimg

        root.after(100, update_frame)

    root.after(0, update_frame)
    root.mainloop()

    if not _stop:
        cleanup()

if __name__ == "__main__":
    main()
