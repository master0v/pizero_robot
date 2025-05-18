#!/usr/bin/env python3
import time
import ultra
from picamera2 import Picamera2, Preview
from PIL import Image, ImageDraw, ImageFont

picam2 = Picamera2()
config = picam2.create_preview_configuration()
picam2.configure(config)
picam2.start_preview(Preview.QTGL)
picam2.start()

# load a font
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 48)

while True:
    dist = ultra.getDistance()
    w, h = picam2.stream_configuration["main"]["size"]
    # make transparent overlay
    img = Image.new("RGBA", (w, h))
    draw = ImageDraw.Draw(img)
    # draw crosshair
    draw.line((w/2, 0, w/2, h), fill="lime", width=4)
    draw.line((0, h/2, w, h/2), fill="lime", width=4)
    # draw centered text
    txt = f"TARGET: {dist:.1f} cm"
    tw, th = draw.textsize(txt, font=font)
    draw.text(((w-tw)/2, (h-th)/2 + 60), txt, font=font, fill="lime")
    # push overlay
    picam2.set_overlay(img.tobytes(), "rgba8888", (w, h))
    time.sleep(0.1)
