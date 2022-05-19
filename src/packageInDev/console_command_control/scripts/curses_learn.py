import curses
import time
import threading

from curses.textpad import Textbox, rectangle
stdscr = curses.initscr()
curses.echo()
# curses.cbreak()
# stdscr.keypad(True)
key = ''

def thread():
    curses.wrapper(input_func)

def input_func(stdscr):
    # Очистить экран
    stdscr.clear()
    while True:
        key = stdscr.getkey()
        stdscr.addstr(0, 0, f"Key: {key}\n")
    
        stdscr.refresh()


if __name__ == "__main__":
    thread_func_of_input = threading.Thread(target=thread, daemon=True)
    thread_func_of_input.start()
    while True:
        print("kkkkkkk: " + str(key))
        time.sleep(1)