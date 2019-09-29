# Automated testing with Optitrack

CSV_PATH="/home/pachacho/Documents/out/kj12e.csv"
MODEL="/home/pachacho/Documents/anafi_tools/processing/models/asfm.model"

# Run Optitrack-ROS1 server in background
rosrun vrpn_node vrpn_server ip:=192.168.0.5 &

load_olympe
load_ros2

ros2 run ros1_bridge dynamic_bridge --all-ros1-to-ros2

# --enable-warning-mode must land the drone if an object is too close
# and can't be avoided
python run_model.py --enable-warning-mode $MODEL $CSV_PATH 