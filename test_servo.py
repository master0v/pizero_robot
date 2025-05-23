#!/usr/bin/env python3
import time
import board
import busio
import curses
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# New bounds and step size
MIN_ANGLE = 30
MAX_ANGLE = 90
STEP = 1       # degrees per keypress
CHANNEL = 11   # PCA9685 channel

def servo_control(stdscr):
    # --- curses setup ---
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0,
        f"Use ↑ to DECREASE and ↓ to INCREASE angle between {MIN_ANGLE}° and {MAX_ANGLE}°, 'q' to quit."
    )
    stdscr.refresh()

    # --- PCA9685 & Servo setup ---
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50
    my_servo = servo.Servo(
        pca.channels[CHANNEL],
        min_pulse=500,
        max_pulse=2500
    )

    # start at the lower bound
    angle = MAX_ANGLE
    my_servo.angle = angle
    stdscr.addstr(2, 0, f"Angle: {angle:.1f}°   ")
    stdscr.refresh()

    try:
        while True:
            key = stdscr.getch()
            if key == curses.KEY_UP:
                # reversed: up arrow now DECREASES angle
                new = max(angle - STEP, MIN_ANGLE)
            elif key == curses.KEY_DOWN:
                # down arrow now INCREASES angle
                new = min(angle + STEP, MAX_ANGLE)
            elif key in (ord('q'), ord('Q')):
                break
            else:
                new = angle

            if new != angle:
                angle = new
                my_servo.angle = angle
                stdscr.addstr(2, 0, f"Angle: {angle:.1f}°   ")
                stdscr.refresh()

            time.sleep(0.05)  # small delay to reduce CPU load

    except KeyboardInterrupt:
        pass

    finally:
        # Clean up curses
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()
        # Detach servo & power down PCA9685
        my_servo.angle = None
        pca.deinit()
        print("Exited. Servo detached and PCA9685 shut down.")

def main():
    curses.wrapper(servo_control)

if __name__ == "__main__":
    main()
