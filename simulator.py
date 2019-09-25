# A test on the tools used here

import cv2

from time import sleep
from scipy import ndimage
from control.controller import AnafiController
from streaming.streaming import StreamProcessing
from teleop.joystick import JoystickTeleop

def edges(cv_frame):
    """
    Applies a Canny (edge-detection) filter
    :param cv_frame: An OpenCV frame with RGB pattern
    """
    # r, g, b = cv2.split(img) then cv2.merge([b,g,r])
    gray = cv2.cvtColor(cv_frame, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    return edges



def id_fun(cv_frame):
    return cv_frame

MP4_FPATH = "Documents/recorded-runs"

if __name__ == "__main__":
    
    cntrl = AnafiController("10.202.0.1")
    strm = StreamProcessing(
        cntrl.drone,
        processing_fun=edges,
        mp4_folderpath=MP4_FPATH
    )
    tjoy = JoystickTeleop(cntrl.drone)

    # Start streaming as soon as the drone takes off
    # Pass control onto the teleop
    # Stop streaming then land the drone
    cntrl.takeoff()
    # cntrl.drone.start_piloting()  # can use drone.piloting_pcmd(roll,pitch,yaw,gaz,tm)
    strm.start()
    
    tjoy.enable()  # runs a loop until "Stop" button is pressed
    # if landed again, stop (if teleop w/ the app)

    strm.stop()
    
    cntrl.landing()
    # cntrl.drone.stop_piloting()
    strm.store()
    strm.open_folder()