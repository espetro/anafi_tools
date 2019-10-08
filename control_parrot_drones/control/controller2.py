
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

class Drone:
    def __init__(self):
        self.drone = olympe.Drone("192.168.42.1")
        self.drone.connection()

    def takeoff(self):
        self.drone(
            TakeOff()
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait()

    def move(self):
        self.drone(
            moveBy(1, 0, 0, 0)
            >> FlyingStateChanged(state="hovering", _timeout=1)
        )

    def land(self):
        self.drone(Landing()).wait()
        self.drone.disconnection()

if __name__ == "__main__":
    dr = Drone()
    dr.takeoff()
    dr.move()
    dr.land()