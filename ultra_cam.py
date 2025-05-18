#!/usr/bin/env python3
import time
import ultra
import numpy as np
from picamera2 import Picamera2, Preview
from PIL import Image, ImageDraw, ImageFont

def main():
    # 1) configure camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480)}
    )
    picam2.configure(config)
    picam2.start_preview(Preview.QT)
    picam2.start()

    # 2) get width & height from your config
    w, h = config["main"]["size"]

    # 3) load a font for drawing distance
    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        32
    )

    try:
        while True:
            # read ultrasonic distance (cm)
            dist = ultra.getDistance()

            # build a transparent RGBA image for the overlay
            img = Image.new("RGBA", (w, h))
            draw = ImageDraw.Draw(img)

            # draw a red crosshair at center
            cx, cy = w // 2, h // 2
            L = 20
            col = (255, 0, 0, 255)
            draw.line([(cx - L, cy), (cx + L, cy)], fill=col, width=3)
            draw.line([(cx, cy - L), (cx, cy + L)], fill=col, width=3)

            # draw the distance text just below it
            txt = f"{dist:.1f} cm"
            tw, th = draw.textsize(txt, font=font)
            draw.text(((w - tw) / 2, cy + L + 10), txt, font=font, fill=col)

            # convert to NumPy and push as the overlay
            overlay = np.array(img, dtype=np.uint8)
            picam2.set_overlay(overlay)

            time.sleep(0.1)   # adjust refresh as needed

    finally:
        picam2.stop_preview()
        picam2.close()

if __name__ == "__main__":
    main()
