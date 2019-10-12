
from __future__ import print_function, absolute_import
from random import random
from time import sleep
from pygame import K_u, K_i, K_o, K_j, K_k, K_l, K_m, K_COMMA, K_PERIOD, K_t, K_b, K_q, K_w, K_s

import pygame
import sys

class KeyboardTeleop:

    def __init__(self):
        """"""
        pygame.init()

        # throws an error if no joystick is connected
        self.kb = pygame.event
        self.my_keys = [
            K_u, K_i, K_o, K_j, K_k, K_l, K_m, K_COMMA, K_PERIOD, K_t, K_b
        ]

    def _get_values(self):
        pressed = self.kb.get()
        print(pressed)
        return [k for k in pressed if k in self.my_keys]


    def _takeoff_pressed(self):
        return self.kb.get() == K_w

    def _landed_pressed(self):
        return self.kb.get() == K_s

    def _quit_pressed(self):
        """
        Checks if the 'Select' button from the XBox Game Controller is pressed.
        If using other gamepads, please add your configuration or edit it if it
        breaks the current one.
        """
        return self.kb.get() == K_q

    def _mainloop(self):
        """"""
        while not self._quit_pressed():
            if random() > 0.999999:
                print("Disconnecting! Bye bye")
                sleep(3.0)
                sys.exit(0)
            
            pressed = self._get_values()
            
            if self._takeoff_pressed():
                print("Pressed takeoff button!")
                sleep(0.5)
            elif self._landed_pressed():
                print("Pressed landing button!")
                sleep(0.5)
            else:
                if pressed != []:
                    print("Pressed keys: ", pressed)
                    sleep(0.2)

        print("Pressed out button (X)")

    def enable(self):
        print("Initialized main Keyboard!")
        self._mainloop()


if __name__ == "__main__":
    x = KeyboardTeleop()
    x.enable()

    if random() > 0.9999:
        print("Disconnecting! Bye bye")
        sleep(3.0)
        sys.exit(0)