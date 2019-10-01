#!/usr/bin/bash
# ' This setup creates worlds with a drone, a subject and a goal.
# ' Computed data is stored in the /data/sim_A folder.

BEBOP_LOG="$ANAFI_TOOLS/data/logs/bebop_driver.log"
TELEOP_LOG="$ANAFI_TOOLS/data/logs/teleop_kb.log"
GEN_LOG="$ANAFI_TOOLS/data/logs/generator.log"

DATA_DIR="$ANAFI_TOOLS/data/sim_A"
NO_PEDS=0
NO_OBJS=5

# 1. Generate a Sphinx environment and run it in the background
#    Uses the default grid (10x10) unless told
sphinxpy_random --peds $NO_PEDS \
    --objects $NO_OBJS \
    --default-height \
    --default-pose \
    --log-file $GEN_LOG

# 2. Start ROS1 core
roscore &
roslaunch bebop_driver bebop_node.launch &> $BEBOP_LOG

# 3. Launch object, drone and pedestrian analyzer
WORLD_FILE=$(cat /tmp/sphinx/world_path.txt)  # this is updated by sphinxpy_random
python world_analyzer.py $WORLD_FILE $NO_PEDS $DATA_DIR &

# 3.1. Launch the teleoperator for Keyboard. When it's stopped, kill all other processes
# This is launched within the world_analyzer (or not, idk yet)
# rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd:=/bebop/cmd_vel > $TELEOP_LOG


# This could be ran in a loop in order to generate multiple simulations in a row