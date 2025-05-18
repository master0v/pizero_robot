#!/usr/bin/env python3
import signal
import time

import ultra
import numpy as np
import RPi.GPIO as GPIO
import cv2

from picamera2 import Picamera2

# —————————————————————————————————————————————
# Globals & signal handling
_stop = False
def _on_sigint(signum, frame):
    global _stop
    _stop = True

signal.signal(signal.SIGINT,  _on_sigint)
signal.signal(signal.SIGTERM, _on_sigint)

# —————————————————————————————————————————————
def main():
    # GPIO setup  
    GPIO.setwarnings(False)
    GPIO.cleanup()

    # Camera setup
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={
        "format": "XRGB8888",   # 4-byte per pixel
        "size":   (640, 480)
    })
    picam2.configure(config)
    picam2.start()

    # OpenCV window
    cv2.namedWindow("UltraCam", cv2.WINDOW_AUTOSIZE)

    try:
        while not _stop:
            # 1) grab a frame
            frame = picam2.capture_array("main")  # H×W×4 RGBA data

            # 2) read distance (in m)
            dist = ultra.getDistance()

            # 3) draw crosshair
            h, w = frame.shape[:2]
            cx, cy, L = w//2, h//2, 20
            col = (0, 0, 255)  # BGR red
            cv2.line(frame, (cx-L, cy), (cx+L, cy), col, 2)
            cv2.line(frame, (cx, cy-L), (cx, cy+L), col, 2)

            # 4) draw text
            txt = f"{dist:.2f} m"
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale, thk = 1.0, 2
            (tw, th), _ = cv2.getTextSize(txt, font, scale, thk)
            x = cx - tw//2
            y = cy + L + th + 10
            cv2.putText(frame, txt, (x, y), font, scale, col, thk, cv2.LINE_AA)

            # 5) show it
            cv2.imshow("UltraCam", frame)
            # waitKey(1) processes X events; returns -1 if no key pressed
            if cv2.waitKey(1) == ord('q'):
                break

            # small sleep to target ~10 Hz
            time.sleep(0.1)

    finally:
        # clean up
        picam2.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
