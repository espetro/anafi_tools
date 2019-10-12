
from __future__ import print_function, absolute_import
from random import random
from time import sleep

import pygame
import sys

class JoystickTeleop:

    def __init__(self):
        """"""
        pygame.init()

        # throws an error if no joystick is connected
        self.joy = pygame.joystick.Joystick(0)  

    def _get_values(self):
        pygame.event.pump()

        if random() > 0.9999:
            print("Axes")
            print([self.joy.get_axis(k) for k in self.joy.get_numaxes()])
            print("Buttons")
            print([self.joy.get_button(k) for k in self.joy.get_numbuttons()])
            sleep(0.3)

        #Read input from the two joysticks and take only the ones we need
        out_joys = [self.joy.get_axis(i) for i in [0,1,3,4]]
        self.joy_values = out_joys

    def _takeoff_pressed(self):
        return self.joy.get_button(3) == 1

    def _landed_pressed(self):
        return self.joy.get_button(0) == 1

    def _quit_pressed(self):
        """
        Checks if the 'Select' button from the XBox Game Controller is pressed.
        If using other gamepads, please add your configuration or edit it if it
        breaks the current one.
        """
        return self.joy.get_button(8) == 1

    def _mainloop(self):
        """"""
        while not self._quit_pressed():
            if random() > 0.99999:
                print("Disconnecting! Bye bye")
                sleep(3.0)
                break
            
            self._get_values()
            
            if self._takeoff_pressed():
                print("Pressed takeoff button!")
                sleep(0.5)
            elif self._landed_pressed():
                print("Pressed landing button!")
                sleep(0.5)
            else:
                print(self.joy_values)
                sleep(0.2)
        print("Pressed out button (X)")

    def enable(self):
        self.joy.init()
        print("Initialized Joystick: {}".format(self.joy.get_name()))

        self._mainloop()


if __name__ == "__main__":
    x = JoystickTeleop()
    x.enable()

    if random() > 0.9999:
        print("Disconnecting! Bye bye")
        sleep(3.0)
        sys.exit(0)