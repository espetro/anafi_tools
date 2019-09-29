#!/usr/bin/python
# Using python2.7 to setup the project

from __future__ import print_function
from setuptools import setup

import os

setup(
    name = "anafi_tools",
    version = "0.0.1",
    author = "Joaquin Terrasa Moya",
    author_email = "quino.terrasa+dev@gmail.com",
    description = ("A set of tools to control, record and test workflows using"
                   "Parrot Anafi and Parrot Bebop2"),
    license = "BSD 3-Clause Revised",
    keywords = "parrot-anafi parrot-bebop2 optitrack ros1 ros2",
    url = "https://github.com/espetro/anafi_tools",
    packages=['processing', 'streaming', 'teleop', 'utils', 'control'],
    long_description=read('README'),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Topic :: Software Development",
        "License :: OSI Approved :: BSD License",
    ],
)

def read(fname):
    """Reads a file inside this directory"""
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


def pip3_pkgs():
    """Prints out python 3 packages to install"""
    print("""pip3 install --user --upgrade:
    numpy
    pandas
    scikit-learn
    torch 
    torchvision
    tensorflow-gpu
    jupyterlab
    """)

def pip2_pkgs():
    """Prints out python 2 packages to install"""
    print("""pip install --user --upgrade:
    numpy
    pandas
    """)

def sim_rl_pkgs():
    """Prints out the packages needed for OptiTrack and Parrot Sphinx"""
    print("""The following packages (urls) are needed to work:
    Olympe (py 3.5.2): https://developer.parrot.com/docs/olympe/installation.html
    Sphinx: https://developer.parrot.com/docs/sphinx/installation.html
    Telemetryd: https://forum.developer.parrot.com/t/datalog-to-a-tcp-read-example/9768/2
    ROS1: ros-kinetic-desktop-full
    ROS2: ros-ardent-desktop
    ROS1-2 bridge: ros-ardent-ros1-bridge
    ROS1 build tool: python-catkin-tools
    ROS2 build tool: python3-colcon-common-extensions
    OptiTrack Motive (licensed, provided by the lab)
    ROS-Optitrack node (using one from IRI, not this): https://wiki.ros.org/vrpn_client_ros
    
    Recommended software:
    VisualStudioCode (snap install --classic code)
    terminator (apt install terminator)

    Also, download drone firmwares to start them faster. Download them to
    $SPHINX_ROOT/firmwares
    bebop2: http://plf.parrot.com/sphinx/firmwares/ardrone3/milos_pc/latest/images/ardrone3-milos_pc.ext2.zip
    anafi4k: http://plf.parrot.com/sphinx/firmwares/anafi/pc/latest/images/anafi-pc.ext2.zip
    """)