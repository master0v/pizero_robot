#!/usr/bin/env python3
"""
ultra_servo_cam.py

Live PiCamera2 → Tkinter video with:
  - ultrasonic distance overlay
  - crosshair
  - servo angle overlay
  - Up/Down keys to drive a PCA9685-driven servo
"""
import os
# ── suppress libcamera INFO/WARN (only ERROR+ appear) ──
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

    # 3) setup Picamera2 → RGB888!
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={
        "format": "RGB888",     # ← now 3-byte RGB
        "size":   (640, 480)
    })
    picam2.configure(config)
    picam2.start()

    # 4) setup Tkinter
    root = tk.Tk()
    root.title("UltraCam + Servo Control")
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

        # grab frame as true RGB
        frame = picam2.capture_array("main")  # shape H×W×3 in RGB
        img = Image.fromarray(frame, mode="RGB")
        draw = ImageDraw.Draw(img)
        w, h = img.size
        cx, cy, L = w // 2, h // 2, 20
        col = (255, 0, 0, 255)

        # crosshair
        draw.line([(cx-L, cy), (cx+L, cy)], fill=col, width=3)
        draw.line([(cx, cy-L), (cx, cy+L)], fill=col, width=3)

        # ultrasonic distance
        dist = ultra.get_distance()
        dist_txt = "Out of range" if dist is None else f"{dist:.2f} m"
        x0, y0, x1, y1 = draw.textbbox((0,0), dist_txt, font=font)
        tw, th = x1-x0, y1-y0
        draw.text(((w-tw)/2, cy+L+10), dist_txt, font=font, fill=col)

        # servo angle only (no instructions)
        angle_txt = f"[↑:↓:Q] Servo: {angle:.1f}°"
        draw.text((10, 10), angle_txt, font=font, fill=col)

        # render to Tk
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
