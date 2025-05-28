#!/usr/bin/env python3
# File name   : control.py
# Description : Curses-driven WASD control (no sudo needed)
# Date        : 2025/05/28

import time
import curses
import move        # your refactored move.py

# full-power range is –100…+100
SPEED = 100

# direct key→(L, R)
KEY_MAP = {
    ord('w'): ( SPEED,  SPEED),
    ord('s'): (-SPEED, -SPEED),
    ord('a'): ( SPEED, -SPEED),
    ord('d'): (-SPEED,  SPEED),
}

def control_loop(stdscr):
    curses.cbreak()           # no Enter needed
    stdscr.nodelay(True)      # non‐blocking getch()
    stdscr.keypad(True)
    stdscr.clear()
    stdscr.addstr(0, 0,
        "Hold W/A/S/D to move, release to stop, CTRL-C=quit",
        curses.A_BOLD
    )

    try:
        while True:
            key = stdscr.getch()

            left, right = KEY_MAP.get(key, (0, 0))
            move.drive(left, right)

            stdscr.addstr(1, 0, f"L={left:>4}  R={right:>4}")
            stdscr.clrtoeol()
            stdscr.refresh()

            time.sleep(0.1)   # 10 ms loop
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    move.setup()
    try:
        curses.wrapper(control_loop)
    finally:
        move.destroy()
