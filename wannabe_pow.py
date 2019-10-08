# How the real-life proof-of-work should go


CONFIG = {
    "runs": 1,
    "object_probs": {"tree": 0.4, "door": 0.0, "wall": 0.2},
    "world_shape": (5,5),
    "number_peds": 1,
    "maximum_objects": 5,
    "subj_to_goal_dist": 4
}

RANDOM_HEIGHT = get_random_height_cmd()

TEST_NAME = "A_environments"  # A = without pedestrians

DATA_DIR = "/home/pachacho/Documents/anafi_tools/data/test"

DRONE_FPATH = "/opt/parrot-sphinx/usr/share/sphinx/drones/local_bebop2.drone"
ACTOR = "/opt/parrot-sphinx/usr/share/sphinx/actors/pedestrian.actor::name={}::path={}"

if __name__ == "__main__":
    
    for i in range(CONFIG["runs"]):
        print_start("Starting run no. {}".format(i))

        world = WorldBuilder(CONFIG)
        objects = world.get_object_models()

        world_fpath, subj_fpath, peds_fpath = world.get_paths()

        subject = ACTOR.format("subject", subj_fpath)
        # nm[-1] gives the pedestrian number; nm ~= P1, P0, ..
        pedestrians = " ".join([ACTOR.format(nm[-1], txt) for (nm, txt) in peds_fpath])

        try:
            BackgroundTask(
                "sphinx {} {} {} {}".format(world_fpath, DRONE_FPATH, subject, pedestrians),
                log=True, log_file="", wait=5
            )

            BackgroundTask("roscore", log=True, log_file="", wait=5)
            
            BackgroundTask(
                "roslaunch bebop_driver bebop_node.launch",
                log=True, log_file="", wait=5
            )
            
            RunTask("rostopic pub --once /bebop/takeoff std_msgs/Empty", wait=5)

            for i in range(5):
                RunTask("rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist {}".format(RANDOM_HEIGHT))

            # TelemetryDaemon(CONFIG) (Aggregator????)

            RunTask("rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/bebop/cmd_vel", wait=5)

        except KeyboardInterrupt as e:
            print_error("CTRL+C has been pressed! Exiting")

            RunTask("rostopic pub --once /bebop/land std_msgs/Empty", wait=5)