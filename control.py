#!/usr/bin/env python3
# File name   : control.py
# Description : Keyboard-driven motor control (hold buttons)
# Date        : 2025/05/23

import time
import curses
import move        # our motor library

# default drive speed (0–100)
SPEED = 80

def control_loop(stdscr):
    """Curses-based real-time WASD control."""
    stdscr.nodelay(True)        # non-blocking getch()
    stdscr.keypad(True)         # enable special keys
    stdscr.clear()
    stdscr.addstr(0, 0,
        "W=↑ forward   S=↓ backward   A=← left   D=→ right   Q=quit",
        curses.A_BOLD
    )

    try:
        while True:
            key = stdscr.getch()

            # quit?
            if key in (ord('q'), ord('Q')):
                break

            # movement only while key is down
            elif key in (ord('w'), ord('W')):
                move.move(SPEED, 'forward', 'no')

            elif key in (ord('s'), ord('S')):
                move.move(SPEED, 'backward', 'no')

            elif key in (ord('a'), ord('A')):
                move.move(SPEED, 'no', 'left')

            elif key in (ord('d'), ord('D')):
                move.move(SPEED, 'no', 'right')

            # no relevant key → stop motors
            else:
                move.motorStop()

            # small delay for responsiveness
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    move.setup()
    try:
        curses.wrapper(control_loop)
    finally:
        move.destroy()
