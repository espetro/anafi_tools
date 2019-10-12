
from __future__ import print_function, absolute_import
from random import random
from time import sleep

import threading
import pygame
import sys

class JoystickTeleop():

    def __init__(self):
        """"""
        self._quit_pressed = False
        self.thread = None

        pygame.init()
        # throws an error if no joystick is connected
        self.joy = pygame.joystick.Joystick(0)  

    def _get_joy_values(self):
        pygame.event.pump()

        #Read input from the two joysticks and take only the ones we need
        out_joys = [self.joy.get_axis(i) for i in [0,1,3,4]]
        return out_joys

    def _is_takeoff_pressed(self):
        return self.joy.get_button(3) == 1

    def _is_landed_pressed(self):
        return self.joy.get_button(0) == 1

    def _check_quit_pressed(self):
        self._quit_pressed = self.joy.get_button(8) == 1

    def _mainloop(self):
        """"""
        while not self._quit_pressed:            
            joy_values = self._get_joy_values()
            
            if self._is_takeoff_pressed():
                print("Pressed takeoff button!")
                sleep(0.5)
            elif self._is_landed_pressed():
                print("Pressed landing button!")
                sleep(0.5)
            else:
                print(joy_values)
                sleep(0.2)
            
            self._check_quit_pressed()

        print("Pressed out button (X)")

    def start(self):
        self.joy.init()
        print("Initialized Joystick: {}".format(self.joy.get_name()))

        self.thread = threading.Thread(target=self._mainloop)
        self.thread.start()

    def stop(self):
        self._quit_pressed = True
        self.thread.join()


if __name__ == "__main__":
    x = JoystickTeleop()
    x.start()

    while x.thread.is_alive():
        sleep(2)
        print("Illo")

    sys.exit(0)