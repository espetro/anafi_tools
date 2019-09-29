#!/usr/bin/python3.5

from streaming.optitrack import OptitrackMRB

def process_data(msg, model, csv_writer, control):
    """
    :param msg: A geometry_msgs/PoseStamped object
    :param csv_writer:
    :param model:
    :param control:
    """
    env_poses = OptiTrackNode.poses2dict(msg)  # that'd be for all drone/peds?
    cmd_vel = model.fit(env_poses)
    control.move(cmd_vel)

    csv_writer.write(env_poses, cmd_vel)

def land_control(event, control):
    """
    :param event:
    :param control:
    """
    control.land()

if __name__ == "__main__":
    CSVPATH = "/home/pachacho/Documents/anafi_tools/data/freeflight/csvname.csv"

    cntrl = AnafiDrone(AnafiDrone.PHYSICAL_IP)
    model = AsfmEngine()
    otrack = OptiTrackNode()
    csv_writer = CsvWriter(CSVPATH)

    otrack.set_callback(lambda m: process_data(m, model, csv_writer, cntrl))
    otrack.on_disconnection(lambda e: land_control(e, cntrl))
    csv_writer.close()
