# How the real-life proof-of-work should go

from gazebo_generator import 

def setupRun():
    """Setups the options for the simulation"""
    return None
    
if __name__ == "__main__":
    
    config = setupRun()

    for i in range(config.runs):
        print_start("Starting run no. {}".format(i))

        world = generate_world(config)
        objects = world.get_object_models()

        try:
            BackgroundTask("sphinx ...", log=True, log_file="", wait=5)
            BackgroundTask("roscore ...", log=True, log_file="", wait=5)
            BackgroundTask("bebop_node ...", log=True, log_file="", wait=5)
            
            RunTask("takeoff ...", wait=5)

            TelemetryDaemon(config)
            RunTask("teleop ...", wait=5)

        except KeyboardInterrupt as e:
            print_error("CTRL+C has been pressed! Exiting")
