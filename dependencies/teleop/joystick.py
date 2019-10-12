
from __future__ import print_function, absolute_import
from random import random
from time import sleep

import threading
import pygame
import sys

try:
    from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
    from olympe.messages.ardrone3.Piloting import TakeOff, Landing
    from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
    import olympe
except ImportError:
    print("Olympe has not been loaded yet! Cannot run the app", file=sys.__stderr__)
    sys.exit(0)


class JoystickTeleop:
    PHYSICAL_IP = "192.168.42.1"
    SIMULATED_IP = "10.202.0.1"
    LAND_TAKEOFF_TIME = 4.0
    MOVE_TIME = 0.1

    def __init__(self, drone=None):
        """"""
        try:
            self._quit_pressed = None
            self.thread = None

            pygame.init()
            self.joy = pygame.joystick.Joystick(0)  

            self.drone = drone
            self.drone.connection()
        except pygame.error as e:
            print(e)
            print("\n(There is no joystick connected to the system)")
            sys.exit(0)

    def _get_joy_values(self):
        pygame.event.pump()

        out_joys = []
        #Read input from the two joysticks and take only the ones we need
        for i in [0,1,3,4]:
            val = self.joy.get_axis(i)
            if val > 0.2:
                out_joys.append(val)
            else:
                out_joys.append(0.0)

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
                self._takeoff()
                sleep(JoystickTeleop.LAND_TAKEOFF_TIME)
            elif self._is_landed_pressed():
                print("Pressed landing button!")
                self._land()
                sleep(JoystickTeleop.LAND_TAKEOFF_TIME)
            else:
                print(joy_values)
                self.move(joy_values)
                sleep(JoystickTeleop.MOVE_TIME)
            
            self._check_quit_pressed()

        print("Pressed QUIT button (X)")

    def _takeoff(self):
        """"""
        print("Takeoff if necessary...")
        self.drone(
            FlyingStateChanged(state="hovering", _policy="check")
            | FlyingStateChanged(state="flying", _policy="check")
            | (
                GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")
                >> (
                    TakeOff(_no_expect=True)
                    & FlyingStateChanged(
                        state="hovering", _timeout=10, _policy="check_wait")
                )
            )
        ).wait()

    def _land(self):
        """Lands the drone"""
        print("Landing...")
        self.drone(
            Landing()
            >> FlyingStateChanged(state="landed", _timeout=5)
        ).wait()

    def move(self, joy_values):
        """
        Move in the desired direction given the (normalized) Joystick values:
        [LeftThumbXAxis, LeftThumbYAxis, RightThumbXAxis, RightThumbYAxis, Select/Quit]
        """
        # movements must be in [-100:100]
        left_right, front_back, turning, up_down = [int(j * 50) for j in joy_values]

        self.drone.piloting_pcmd(
            -int(left_right), int(front_back), int(turning), int(up_down),
            1
        )

    def start(self):
        self.joy.init()
        print("Initialized Joystick: {}".format(self.joy.get_name()))
        
        self.drone.start_piloting()

        self.thread = threading.Thread(target=self._mainloop)
        self.thread.start()

    def stop(self):
        self._quit_pressed = True
        self.thread.join()

        self.drone.stop_piloting()
        self.drone.disconnection()


if __name__ == "__main__":
    drone = olympe.Drone(JoystickTeleop.SIMULATED_IP, loglevel=0)

    x = JoystickTeleop(drone)
    x.start()

    while x.thread.is_alive():
        x = 1

    sys.exit(0)