#!/usr/bin/env python3
"""
ultra.py

HC-SR04 reader with:
 - single init, monotonic timing, timeouts, clean exit
 - median-of-N filtering + exponential smoothing
 - temperature compensation
 - recommended ≥60 ms between pulses
"""

import RPi.GPIO as GPIO
import time
from statistics import median

# GPIO pins (BCM numbering)
TRIG = 11
ECHO = 8

# Constants
TIMEOUT = 0.05              # seconds to wait for echo (~8.5 m max)
MIN_INTERVAL = 0.06         # minimum 60 ms between triggers
SAMPLE_COUNT = 7            # number of raw readings per filtered measurement
SAMPLE_DELAY = 0.01         # 10 ms between raw readings
ALPHA = 0.2                 # smoothing factor (0<α<1)

# Globals for smoothing
_smoothed = None
_last_trigger = 0.0

def init_sensor():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO, GPIO.IN)
    # let sensor settle
    time.sleep(0.1)

def read_temperature_celsius() -> float:
    """
    Stub for ambient temperature reading.
    Replace this with your own sensor code (e.g. DHT22, DS18B20).
    """
    return 24.4  # assume 24.4 °C / 76 F if no sensor attached

def get_distance(timeout: float = TIMEOUT) -> float | None:
    """
    Send a 10 μs pulse on TRIG, measure flight time on ECHO.
    Returns distance in metres, or None on timeout.
    """
    global _last_trigger
    now = time.monotonic()
    # Enforce minimum interval to avoid echo overlap
    wait = MIN_INTERVAL - (now - _last_trigger)
    if wait > 0:
        time.sleep(wait)
    _last_trigger = time.monotonic()

    # 1) Trigger pulse
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(10e-6)
    GPIO.output(TRIG, GPIO.LOW)

    # 2) Wait for ECHO high
    t_start = time.monotonic()
    while GPIO.input(ECHO) == 0:
        if time.monotonic() - t_start >= timeout:
            return None
    t0 = time.monotonic()

    # 3) Wait for ECHO low
    while GPIO.input(ECHO) == 1:
        if time.monotonic() - t0 >= timeout:
            return None
    t1 = time.monotonic()

    # 4) Compute distance with temperature compensation
    T = read_temperature_celsius()
    v_sound = 331.4 + 0.6 * T    # m/s
    return (t1 - t0) * v_sound / 2.0

def get_filtered_distance() -> float | None:
    """
    Take SAMPLE_COUNT readings, return the median of the valid ones.
    """
    readings = []
    for _ in range(SAMPLE_COUNT):
        d = get_distance()
        if d is not None:
            readings.append(d)
        time.sleep(SAMPLE_DELAY)
    if not readings:
        return None
    return median(readings)

def get_stable_distance() -> float | None:
    """
    Apply exponential smoothing to the median-filtered reading.
    """
    global _smoothed
    new = get_filtered_distance()
    if new is None:
        return None
    if _smoothed is None:
        _smoothed = new
    else:
        _smoothed = ALPHA * new + (1 - ALPHA) * _smoothed
    return _smoothed

def main():
    init_sensor()
    print(get_distance())
    print(get_filtered_distance())
    dist = get_stable_distance()
    if dist is None:
        print("Out of range")
    else:
        print(f"{dist:.2f} m")
    # Main loop rate; effectively ~1 Hz (includes internal waits)
    time.sleep(0.1)
    GPIO.cleanup()

if __name__ == "__main__":
    main()
