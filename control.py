#!/usr/bin/env python3
# File name   : control.py
# Description : Keyboard-driven motor control
# Author      : You
# Date        : 2025/05/23

import time
import curses
import move        # our motor library

# default speed (0–100)
SPEED = 50

def control_loop(stdscr):
    """Curses-based real-time keyboard control."""
    stdscr.nodelay(True)
    stdscr.keypad(True)
    stdscr.clear()
    stdscr.addstr(0, 0,
        "Arrows: ↑↓←→  Space: stop  Q: quit", curses.A_BOLD)

    try:
        while True:
            key = stdscr.getch()
            if key == curses.KEY_UP:
                move.move(SPEED, 'forward', 'no')
            elif key == curses.KEY_DOWN:
                move.move(SPEED, 'backward', 'no')
            elif key == curses.KEY_LEFT:
                move.move(SPEED, 'no', 'left')
            elif key == curses.KEY_RIGHT:
                move.move(SPEED, 'no', 'right')
            elif key == ord(' ') or key == ord('s'):
                move.motorStop()
            elif key in (ord('q'), ord('Q')):
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    move.setup()
    try:
        curses.wrapper(control_loop)
    finally:
        move.destroy()
