#!/usr/bin/env python3
import time
import board
import busio
import curses
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Bounds and step size
MIN_ANGLE = 28
MAX_ANGLE = 90
STEP = 1    # degrees per keypress
CHANNEL = 11

def servo_control(stdscr):
    # --- curses setup ---
    curses.noecho()            # don’t echo pressed keys
    curses.cbreak()            # react to keys instantly
    stdscr.keypad(True)        # capture special keys
    stdscr.nodelay(True)       # non-blocking getch()
    stdscr.clear()
    stdscr.addstr(0, 0,
        f"Use ↑/↓ to move servo between {MIN_ANGLE}° and {MAX_ANGLE}°, 'q' to quit."
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
    angle = MIN_ANGLE
    my_servo.angle = angle
    stdscr.addstr(2, 0, f"Angle: {angle:.1f}°   ")
    stdscr.refresh()

    try:
        while True:
            key = stdscr.getch()
            if key == curses.KEY_UP:
                new = min(angle + STEP, MAX_ANGLE)
            elif key == curses.KEY_DOWN:
                new = max(angle - STEP, MIN_ANGLE)
            elif key in (ord('q'), ord('Q')):
                break
            else:
                new = angle

            if new != angle:
                angle = new
                my_servo.angle = angle
                stdscr.addstr(2, 0, f"Angle: {angle:.1f}°   ")
                stdscr.refresh()

            # small delay so we don't hammer the CPU
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    finally:
        # restore terminal
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()
        # detach servo & power down PCA9685
        my_servo.angle = None
        pca.deinit()
        print("Exited. Servo detached and PCA9685 shut down.")

def main():
    curses.wrapper(servo_control)

if __name__ == "__main__":
    main()
