#!/usr/bin/env python3
import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Minimum and maximum angles your servo can safely reach
MIN_ANGLE = 28   # new lower bound
MAX_ANGLE = 90   # upper bound remains

def main():
    # 1) Initialize I2C & PCA9685
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50  # Standard for most hobby servos

    # 2) Create a Servo on channel 11
    my_servo = servo.Servo(
        pca.channels[11],
        min_pulse=500,   # microseconds
        max_pulse=2500   # microseconds
    )

    print(f"Servo control ready. Enter an angle between {MIN_ANGLE}° and {MAX_ANGLE}°, or 'q' to quit.")
    try:
        while True:
            user = input(f"Angle ({MIN_ANGLE}–{MAX_ANGLE}) or 'q': ").strip().lower()
            if user == 'q':
                print("Exiting.")
                break

            try:
                angle = float(user)
            except ValueError:
                print(f"  ↳ Please enter a number between {MIN_ANGLE} and {MAX_ANGLE}, or 'q'.")
                continue

            if not MIN_ANGLE <= angle <= MAX_ANGLE:
                print(f"  ↳ Out of range. Enter an angle from {MIN_ANGLE} to {MAX_ANGLE}.")
                continue

            print(f"  ↳ Moving servo to {angle:.1f}°.")
            my_servo.angle = angle
            time.sleep(0.5)  # give it a moment to move

    except KeyboardInterrupt:
        print("\nInterrupted — stopping.")

    finally:
        # Clean up
        my_servo.angle = None    # detach servo
        pca.deinit()             # turn off the PCA9685
        print("Cleanup done. Goodbye!")

if __name__ == "__main__":
    main()
