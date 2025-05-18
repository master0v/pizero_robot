#!/usr/bin/env python3
"""
ultra_cam.py

Live PiCamera2 → Tkinter video with ultrasonic overlay.
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

# ────────────────────────────────────────────────────────
# Graceful shutdown
_stop = False
def _on_sigint(signum, frame):
    global _stop
    _stop = True

signal.signal(signal.SIGINT,  _on_sigint)
signal.signal(signal.SIGTERM, _on_sigint)

def main():
    # 1) initialize your ultrasonic GPIO once
    ultra.init_sensor()

    # 2) camera setup
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={
        "format": "XRGB8888",
        "size":   (640, 480)
    })
    picam2.configure(config)
    picam2.start()

    # 3) Tkinter window
    root = tk.Tk()
    root.title("UltraCam")
    canvas = tk.Canvas(root, width=640, height=480)
    canvas.pack()

    # 4) preload font
    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 32
    )

    def update_frame():
        if _stop:
            root.destroy()
            return

        # grab a frame
        frame = picam2.capture_array("main")  # H×W×4

        # build PIL image & draw
        img = Image.fromarray(frame[:, :, :3])
        draw = ImageDraw.Draw(img)
        w, h = img.size
        cx, cy, L = w // 2, h // 2, 20
        col = (255, 0, 0, 255)

        # crosshair
        draw.line([(cx-L, cy), (cx+L, cy)], fill=col, width=3)
        draw.line([(cx, cy-L), (cx, cy+L)], fill=col, width=3)

        # distance
        dist = ultra.get_distance()  # metres or None
        txt = "Out of range" if dist is None else f"{dist:.2f} m"
        x0, y0, x1, y1 = draw.textbbox((0, 0), txt, font=font)
        tw, th = x1 - x0, y1 - y0
        draw.text(((w - tw) / 2, cy + L + 10), txt, font=font, fill=col)

        # convert to PPM and feed to Tk
        buf = io.BytesIO()
        img.save(buf, format="PPM")
        img_data = buf.getvalue()
        tkimg = tk.PhotoImage(data=img_data)

        canvas.create_image(0, 0, anchor="nw", image=tkimg)
        canvas.img = tkimg  # keep ref

        root.after(100, update_frame)  # ~10 Hz

    root.after(0, update_frame)
    root.mainloop()

    # teardown
    picam2.stop()
    picam2.close()
    GPIO.cleanup()

if __name__ == "__main__":
    main()
