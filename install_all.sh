#!/usr/bin/bash
# Installs the following tools or packages:
# Parrot Olympe, Parrot Sphinx, ROS1, ROS2
# Tensorflow, PyTorch, JupyterLab, CUDA-cuDNN
# Parrot Bebop2-Anafi control packages
# + devtools (VsCode, PyCharm, Opera, Terminator)
# + updates the system (Ubuntu 16.04)

# Check if SUDO - this script CANT be run with SUDO (it uses $HOME)
if ["$SEUID" -e 0]; then
  echo -e "Please call this script without 'sudo'"
  exit
fi

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Adds RED and NOCOLOR constants for echoing
RED='\033[0;31m' 
NC='\033[0m'

# 1. Install configuration tools used for installing main pkgs or development
echo -e "${RED}Installing configuration and build tools${NC}"
sudo apt update 
sudo apt install curl gnupg2 lsb-release g++ \
  freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev \
  libglu1-mesa libglu1-mesa-dev wget snapd terminator python-pip \
  python3-pip intel-microcode git compizconfig-settings-manager \
  -y

# =====================
echo "alias refreshenv='source $HOME/.bashrc'" >> $HOME/.bashrc
source $HOME/.bashrc
# =====================

# ROS2 setup
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# ROS1 setup
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Sphinx
echo "deb http://plf.parrot.com/sphinx/binary `lsb_release -cs`/" | sudo tee /etc/apt/sources.list.d/sphinx.list > /dev/null
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 508B1AE5


# =====================
sudo apt-get update
# =====================


# =====================
# 2. ROS1
echo -e "${RED}Installing ROS1 (Kinetic)${NC}"
sudo apt-get install ros-kinetic-desktop-full python-catkin-tools ros-kinetic-teleop-twist-keyboard -y
echo "source /opt/ros/kinetic/setup.bash  # load ROS1" >> $HOME/.bashrc  # ROS1 is loaded by default
# Adds a teleop command utility when running ROS1 + bebop_autonomy node
echo "alias load_teleop_kb='rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/bebop/cmd_vel'" >> $HOME/.bashrc
echo -e "${RED}REMEMBER TO CLONE AND BUILD BEBOP_AUTONOMY NODE!!${NC}"


# =====================
# 3. ROS2 ardent + ROS1_bridge
echo -e "${RED}Installing ROS2 (Ardent) and ROS1 Bridge${NC}"
sudo apt install ros-ardent-desktop python3-colcon-common-extensions ros-ardent-ros1-bridge -y

echo "alias load_ros2='source /opt/ros/ardent/setup.bash'" >> $HOME/.bashrc  # ROS2 is loaded on demand


# =====================
# 4. Parrot Sphinx (needs to add yourself to 'firmwared' group)
echo -e "Installing Parrot Sphinx. ${RED}NOTE! You need to manually accept the license and add you to the firmwared group${NC}"
sudo apt-get install parrot-sphinx -y 

sudo systemctl stop firmwared.service
sudo systemctl start firmwared.service
sudo systemctl enable firmwared.service  # firmwared will be ran at startup

echo "export SPHINX_ROOT='/opt/parrot-sphinx/usr/share/sphinx/'" >> $HOME/.bashrc
refreshenv

sudo chmod a+x $SPHINX_ROOT  # gives access to sphinx folder, which contains the worlds and models
mkdir -p $SPHINX_ROOT/firmwares
cd $SPHINX_ROOT/firmwares
# Download both Parrot Anafi and Parrot Bebop2 firmwares to local so to run Sphinx faster
wget http://plf.parrot.com/sphinx/firmwares/ardrone3/milos_pc/latest/images/ardrone3-milos_pc.ext2.zip
wget http://plf.parrot.com/sphinx/firmwares/anafi/pc/latest/images/anafi-pc.ext2.zip
cd  # return to $HOME


# =====================
# 5. Parrot Olympe
echo -e "Installing Parrot Olympe.${RED}NOTE! Olympe only works in Python 3.5.2.${NC}"
cd $HOME
mkdir -p $HOME/Documents/parrot/groundsdk
cd $HOME/Documents/parrot/groundsdk

git config --global user.email "quino.terrasa+dev@gmail.com"
git config --global user.name "espetro"

repo init -u https://github.com/Parrot-Developers/groundsdk-manifest.git
repo sync
y  # "enable color display in this user account"

bash products/olympe/linux/env/postinst  # install dependencies
bash build.sh -p olympe-linux -A all final -j  # build olympe-linux

echo "export OLYMPE_ROOT='$HOME/Documents/parrot/groundsdk'" >> $HOME/.bashrc
echo "alias load_olympe='source $($OLYMPE_ROOT)/products/olympe/linux/env/setenv'" >> $HOME/.bashrc  # Olympe is loaded on demand
refreshenv
cd  # return to $HOME


# =====================
# 6. DevTools
echo -e "${RED}Installing developer tools${NC}"
sudo snap install opera
sudo snap install code --classic
sudo snap install pycharm-community --classic
sudo apt install terminator nano -y


# =====================
# (7. Optional) CUDA 9.0.* + cuDNN 7.* + NVIDIA 384.130 driver
# I'm using NVIDIA GTX 660
# Check the driver in Ubuntu's 'Additional Drivers' GUI.
read -n1 -p "Install NVIDIA+CUDA dev toolkit? [Y, n]: " DO_THAT

case $DO_THAT in
  y|Y) echo -e "Installing NVIDIA+CUDA development toolkit.${RED}BE AWARE! You need an NVIDIA GPU compatible with CUDA 9.0 or greater.${NC}" ;;
  n|N) echo "CUDA installation skipped" ;;
  *) echo "Wrong input" ;;
esac

# cd $HOME/Downloads
# wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-repo-ubuntu1604_9.0.176-1_amd64.deb
# sudo dpkg -i cuda-repo-ubuntu1604_9.0.176-1_amd64.deb
# sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub
# sudo apt-get update
# sudo apt-get install cuda-9.0

# export PATH="$PATH:/usr/local/cuda-9.0/bin" >> $HOME/.bashrc


# =====================
# (8. Optional) Install Tensorflow or/and PyTorch with GPU support.
## echo "Installing TF and PyTorch in GPU versions.${RED}NOTE! The Olympe environment (Py3.5.2) will be used.${NC}"
## load_olympe
## cd $HOME/Downloads
## sudo apt install python3-pip -y  # check if not installed for Olympe environment

# Install PyTorch 1.1.0 for Py3.5 with CUDA 9.0
## wget https://download.pytorch.org/whl/cu90/torch-1.1.0-cp35-cp35m-linux_x86_64.whl
## pip3 install torch-1.1.0-cp35-cp35m-linux_x86_64.whl
# Install Tensorflow 1.* for Py3.5 with GPU support (not CUDA version specified)
## pip3 install --upgrade tensorflow-gpu

## cd  # return to $HOME

# =====================
# 9. Install Jupyter-lab
echo -e "${RED} Installing Jupyter Lab on Olympe environment${NC}"
load_olympe
pip3 install jupyterlab

# =====================
echo -e "${RED}Please reboot${NC}"

# overwrites the configuration for not getting the "System problem detected" window.
# telemetryd process returns A LOT of these windows over time. Edit it with 'enabled=1' if you want to re-enable it.
sudo echo "enabled=0" > /etc/default/apport
# =====================

