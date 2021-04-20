#!/usr/bin/env python
#-*- coding: utf-8 -*-

import threading
import queue

import pynput.keyboard as kbd
from pynput.keyboard import Listener


def read_kbd_input(input_queue):
    """Keyboard thread's run function reads keyboard when key is pressed."""
    def on_press(key):
        # stop if esc pressed
        if key == kbd.Key.esc:
            return False  # Stop listener
        # put key in queue
        input_queue.put(key)
        
    print('Ready for keyboard input:')
    # keyboard listener loop (provide callback)
    with Listener(on_press=on_press) as listener:
        listener.join()


class SerialReadIn:
    """Reads commands from the keyboard."""

    def __init__(self):
        self.input_queue = queue.Queue()
        self.inputThread = threading.Thread(target=read_kbd_input, args=(self.input_queue,))#, daemon=True)
        self.inputThread.start()


if __name__ == '__main__':
    s = SerialReadIn()
    while True:
        if s.input_queue.qsize() > 0:
            input_str = s.input_queue.get()
            print("got: {}".format(input_str))
