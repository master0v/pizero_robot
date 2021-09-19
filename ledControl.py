#!/usr/bin/env python3
#  -*- coding: utf-8 -*-
# from https://github.com/rpi-ws281x/rpi-ws281x-python/tree/master/examples

import time
from rpi_ws281x import PixelStrip, Color

class ledControl():
  def __init__(self, *args, **kwargs):
    # LED strip configuration:
    self.LED_COUNT = 12 # Number of LED pixels.
    self.LED_PIN = 12          # GPIO pin connected to the pixels (18 uses PWM!).
    # LED_PIN = 10        # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
    self.LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
    self.LED_DMA = 10          # DMA channel to use for generating signal (try 10)
    self.LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
    self.LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
    self.LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

    # Create NeoPixel object with appropriate configuration.
    self.strip = PixelStrip(
        self.LED_COUNT,
        self.LED_PIN,
        self.LED_FREQ_HZ,
        self.LED_DMA,
        self.LED_INVERT,
        self.LED_BRIGHTNESS,
        self.LED_CHANNEL
      )
    # Intialize the library (must be called once before other functions).
    self.strip.begin()

    self.RED = Color(255, 0, 0)
    self.GREEN = Color(0, 255, 0)
    self.BLUE = Color(0, 0, 255)
    self.BLACK = Color(0, 0, 0)


  # Define functions which animate LEDs in various ways.
  def colorWipe(self, strip, color, wait_ms=50):
      """Wipe color across display a pixel at a time."""
      for i in range(int(strip.numPixels()/2)):
          strip.setPixelColor(i, color)
          strip.setPixelColor(i+6, color)
          strip.show()
          time.sleep(wait_ms / 1000.0)


  def theaterChase(self, strip, color, wait_ms=50, iterations=10):
      """Movie theater light style chaser animation."""
      for j in range(iterations):
          for q in range(3):
              for i in range(0, strip.numPixels(), 3):
                  strip.setPixelColor(i + q, color)
              strip.show()
              time.sleep(wait_ms / 1000.0)
              for i in range(0, strip.numPixels(), 3):
                  strip.setPixelColor(i + q, 0)


  def wheel(self, pos):
      """Generate rainbow colors across 0-255 positions."""
      if pos < 85:
          return Color(pos * 3, 255 - pos * 3, 0)
      elif pos < 170:
          pos -= 85
          return Color(255 - pos * 3, 0, pos * 3)
      else:
          pos -= 170
          return Color(0, pos * 3, 255 - pos * 3)


  def rainbow(self, strip, wait_ms=20, iterations=1):
      """Draw rainbow that fades across all pixels at once."""
      for j in range(256 * iterations):
          for i in range(strip.numPixels()):
              strip.setPixelColor(i, wheel((i + j) & 255))
          strip.show()
          time.sleep(wait_ms / 1000.0)


  def rainbowCycle(self, strip, wait_ms=20, iterations=5):
      """Draw rainbow that uniformly distributes itself across all pixels."""
      for j in range(256 * iterations):
          for i in range(strip.numPixels()):
              strip.setPixelColor(i, wheel(
                  (int(i * 256 / strip.numPixels()) + j) & 255))
          strip.show()
          time.sleep(wait_ms / 1000.0)


  def theaterChaseRainbow(self, strip, wait_ms=50):
      """Rainbow movie theater light style chaser animation."""
      for j in range(256):
          for q in range(3):
              for i in range(0, strip.numPixels(), 3):
                  strip.setPixelColor(i + q, wheel((i + j) % 255))
              strip.show()
              time.sleep(wait_ms / 1000.0)
              for i in range(0, strip.numPixels(), 3):
                  strip.setPixelColor(i + q, 0)
                  
  def redColorWipe(self, wait_ms=100):
    self.colorWipe(self.strip, self.RED, wait_ms)  # Red wipe


  def greenColorWipe(self, wait_ms=100):
    self.colorWipe(self.strip, self.GREEN, wait_ms)  # Red wipe
    
    
  def blueColorWipe(self, wait_ms=100):
    self.colorWipe(self.strip, self.BLUE, wait_ms)  # Red wipe


  def wipeClean(self):
    self.colorWipe(self.strip, self.BLACK, 50)


# Main program logic follows:
if __name__ == '__main__':
    print('Press Ctrl-C to quit.')
    lc=ledControl()
    try:
        while True:
            print('Color wipe animations.')
            lc.redColorWipe()
            lc.wipeClean()
            lc.greenColorWipe()
            lc.wipeClean()
            lc.blueColorWipe()
            lc.wipeClean()
            #colorWipe(strip, BLUE, 100)  # Blue wipe
            # print('Theater chase animations.')
            # theaterChase(strip, Color(127, 127, 127))  # White theater chase
            # theaterChase(strip, Color(127, 0, 0))  # Red theater chase
            # theaterChase(strip, Color(0, 0, 127))  # Blue theater chase
            # print('Rainbow animations.')
            # rainbow(strip)
            # rainbowCycle(strip)
            # theaterChaseRainbow(strip)

    except KeyboardInterrupt:
      lc.wipeClean()