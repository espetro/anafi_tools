
import pygame

from olympe.messages.ardrone3.Piloting import moveBy

class JoystickTeleop:

    def __init__(self, drone):
        """"""
        pygame.init()

        self.drone = drone
        # throws an error if no joystick is connected
        self.joy = pygame.joystick.Joystick(0)  

    def _get_values(self):
        pygame.event.pump()

        #Read input from the two joysticks and take only the ones we need
        out_joys = [self.joy.get_axis(i) for i in [0,1,3,4]]
        self.joy_values = out_joys

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
            self._get_values()
            self._move()

    def _move(self):
        """
        Move in the desired direction given the (normalized) Joystick values:
        [LeftThumbXAxis, LeftThumbYAxis, RightThumbXAxis, RightThumbYAxis, Select/Quit]
        """
        left_right, front_back, turning, up_down = self.joy_values
        THRESHOLD = 0.3

        if round(abs(left_right), 2) < THRESHOLD:
            left_right = 0.0
        if round(abs(front_back), 2) < THRESHOLD:
            front_back = 0.0
        if round(abs(turning), 2) < THRESHOLD:
            turning = 0.0
        if round(abs(up_down), 2) < THRESHOLD:
            up_down = 0.0

        self.drone(moveBy(-front_back, left_right, up_down, turning,_timeout=5)) \
            .wait() \
            .success()

    def enable(self):
        self.joy.init()
        print("Initialized Joystick: {}".format(self.joy.get_name()))

        self._mainloop()


# if __name__ == "__main__":
#     x = JoystickTeleop()
#     x.enable()