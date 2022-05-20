#!/usr/bin/env python3
import curses
import time
import threading
import os

from curses.textpad import Textbox, rectangle
stdscr = curses.initscr()
# curses.echo()
# curses.cbreak()
# stdscr.keypad(True)
pad = curses.newpad(100, 100)
#  These loops fill the pad with letters; this is
# explained in the next section
for y in range(0, 100):
    for x in range(0, 100):
        try: 
            pad.addch(y,x, 'a')
        except curses.error: 
            pass

#  Displays a section of the pad in the middle of the screen
pad.refresh( 0,0, 5,5, 20,75)