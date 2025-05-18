#!/usr/bin/env python3
"""
ultra.py

HC-SR04 reader: single init, monotonic timing, timeouts, and clean exit.
"""

import RPi.GPIO as GPIO
import time

# GPIO pins (BCM numbering)
TRIG = 11
ECHO = 8

# How long to wait for the echo (seconds); ~0.05s ⇒ max ~8.5 m
TIMEOUT = 0.05

def init_sensor():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO, GPIO.IN)
    # let sensor settle
    time.sleep(0.1)

def get_distance(timeout: float = TIMEOUT) -> float | None:
    """
    Send a 10 µs pulse on TRIG, measure flight time on ECHO.
    Returns:
        distance in metres, or None if no echo within `timeout`.
    """
    # 1) Trigger pulse
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(10e-6)
    GPIO.output(TRIG, GPIO.LOW)

    # 2) Wait for ECHO to go HIGH
    t_start = time.monotonic()
    while GPIO.input(ECHO) == 0:
        if time.monotonic() - t_start >= timeout:
            return None
    t0 = time.monotonic()

    # 3) Wait for ECHO to go LOW
    while GPIO.input(ECHO) == 1:
        if time.monotonic() - t0 >= timeout:
            return None
    t1 = time.monotonic()

    # 4) Compute distance (v_sound ≈ 343 m/s)
    return (t1 - t0) * 343.0 / 2.0

def main():
    init_sensor()
    try:
        while True:
            dist = get_distance()
            if dist is None:
                print("Out of range")
            else:
                print(f"{dist:.2f} m")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
