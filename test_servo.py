#!/usr/bin/env python3
import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

def main():
    # 1) Initialize I2C & PCA9685
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50  # Standard for most hobby servos

    # 2) Create a Servo on channel 11
    #    (You can optionally tweak min_pulse and max_pulse if your servo needs it)
    my_servo = servo.Servo(
        pca.channels[11],
        min_pulse=500,   # in microseconds
        max_pulse=2500
    )

    print("Servo sweep starting. Ctrl+C to stop.")
    try:
        # 3) Sweep between 0° and 90°
        while True:
            my_servo.angle = 0
            time.sleep(1)
            my_servo.angle = 90
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nInterrupted by user — stopping sweep.")

    finally:
        # 4) Clean up
        my_servo.angle = None    # detach servo
        pca.deinit()             # turn off the PCA9685
        print("Cleanup done. Goodbye!")

if __name__ == "__main__":
    main()
