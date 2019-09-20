
import olympe

from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt, setAutonomousFlightMaxHorizontalSpeed, setAutonomousFlightMaxVerticalSpeed
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged

class AnafiController:

    def __init__(self, ip="10.202.0.1"):
        self.drone = olympe.Drone(
            ip,
            loglevel=4,
            # debug= 4, info= 3, warning= 2, error= 1, critical= 0
        )

        # ...

        # set tilt after calling
        self._start()

    def _start(self):
        """"""
        self.drone.connection()

    def _stop(self):
        """"""
        self.drone.disconnection()

    def takeoff(self):
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

        self.drone(MaxTilt(40)).wait().success()
        self.drone(setAutonomousFlightMaxVerticalSpeed(10)).wait().success()
        self.drone(setAutonomousFlightMaxVerticalSpeed(10)).wait().success()

    def move(self, lr, fb, tlr, ud):
        """
        Move the drone in the allowed space
        :param lr: Move to the left (-1) or right (1)
        :param fb: Move to the front (1) or back (-1)
        :param tlr: Turn to the left (-1) or right (1)
        :param ud: Go up (1) or down (-1)
        """
        self.drone(moveBy(-fb, lr, ud, tlr, _timeout=10)) \
            .wait() \
            .success()

    def landing(self):
        """Lands the drone"""
        print("Landing...")
        self.drone(
            Landing()
            >> FlyingStateChanged(state="landed", _timeout=5)
        ).wait()
        
        self._stop()


# Desired functionality
# cntrl = AnafiController(drone)
# cntrl.takeoff()

# while not (is_goal_over() or did_fail()):
#     cntrl.get(update_object_from_processing) \
#         .then(move) \
#         .catch(landing)  # in an infinite loop or until goal is not met
