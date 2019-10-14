
# Parrot Anafi Tools

![A colored badge](https://img.shields.io/pypi/v/hello1234?color=green&label=anafi_tools&style=flat-square)

A set of scripts and bootstrapped API-like interfaces to help quickstart a project involving the Parrot Anafi drone. As of now, this package is focused on the simulation of scenarios with the Anafi, taking care of world random generation, teleoperation and automatic control of the drone, as well as environment data streaming, collection and processing.

Moreover, an interface is provided to execute constrained simulations in an easy, hassle-free way.

This package supports both Python 2 and Python 3.


## Required pre-configuration

```py
# PLEASE PROVIDE THE PARROT GROUNDSDK COMMON DIRECTORY
PARROT_COMMON = "/Documents/parrot/groundsdk/packages/common"
# PARROT_COMMON = "/Documents/code/parrot-groundsdk/packages/common"
POMP_LIB = os.path.expanduser("~") + PARROT_COMMON + "/libpomp/python"
TELEMETRYD_LIB = os.path.expanduser("~") + PARROT_COMMON + "/telemetry/tools"
```

## Quickstart

[...]

---

## Technology stack

Due to personal project requirements, there is a lot of libraries entangled in the dark bays of the codebase :smile: :smile:. Moreover, as some legacy tools that are available to old, widely-tested drones such as the Parrot Bebop 2, and could be extended to Parrot Anafi were initially intended to be used in the project, there may be still some remains of them walking around, so don't bother to open an issue if you find it :smirk:.

The current list of libraries used for this project are:
  + ROS1 (kinetic)
  + ROS2 (ardent)
  + PyGame
  + Tensorflow
  + Gazebo / Parrot Sphinx
  + Parrot Olympe

Moreover, the list of used (or installed) packages is available at `requirements.txt`.

## Modules

The tools are organized into modules, and some of them are ready to be configured as python-pip packages. Note that most of them are a mix of Python scripts, bash scripts and data files, such as `.csv`s or serialized formats.

This is the folder structure you can find in the root of the project:

```
├── control
│   ├── drone: Interfaces the drone control by providing it a velocity command.
│   ├── notebooks
│   ├── README.md
│   └── teleop: Provides easy-to-use Joystick and Keyboard controllers.
|
├── data
│   ├── bebop2: Example data logged using the Bebop2 + ROS1 stack.
│   ├── camera: Logs recorded with the simulated drone camera.
│   ├── rosbag: ROSbag serialized data that simulates the ROS streaming system used to get drone data.
│   ├── test: Data recorded at the test sessions with the drone.
│   └── train: Data recorded at the training sessions with the drone.
│
├── dependencies
│   ├── catkinws_src.tar.gz: ROS1-2 source references of the packages used.
│   ├── gz_models: Gazebo models used in the random world generation.
│   └── sphinx.tar.gz: Configuration files for Parrot Sphinx.
│
├── engines: The neural models used to automate drone velocity.
│
├── envdata
│   ├── aggregate: Holds common functions used for both systems.
│   ├── optitrack: Interfaces the OptiTrack data streaming.
│   ├── README.md
│   └── telemetry: Interfaces the Sphinx Telemetry data streaming.
│
├── gazebo_world_gen: A python-pip package to generate grid-like constrained random worlds.
|   |
│   ├── example: An example of a generated world.
|   |
│   ├── generators: Set of scripts used to generate a world and format it using Gazebo rules.
│   ├── MANIFEST.in
│   ├── models: Object models found in the world.
│   ├── README.md
│   ├── setup.py
│   └── test.py
├── LICENSE: The project's license, extended from used software licenses.
|
├── pipelines: Workflows done using the project modules.
|   └──utils: Tools common to all workflows.
|
├── README.md
├── requirements.txt
└── setup.py
```

The following modules can be installed as packages:
  + control
  + envdata
  + gazebo-world-gen

Thus, you can use them without path-appending the module path to the mainfile. To do it, run `pip install -e .` in the module's root folder. Bear in mind that all packages have been re-configured to be used in the Olympe environment, thus you have to source it first (you can do it by running `source $GROUNDSDK_ROOT/products/olympe/linux/env/setenv`). Then you can import modules from e.g. `control` like this:

```python
from telemetry.data_logger import DataLogger
from teleop.joystick import JoystickTeleop
```

### A common workflow 

The execution of the application is described at `/pipelines/expert_policies.py`. Here, we quickstart the whole system by invoking the multiple modules using a common configuration. This configuration can be found in `utils/configs.py`, where a setup configuration is defined for each desired run, either simulated or not.


[parrot_dev]: https://developer.parrot.com/
[wifi_control]: https://developer.parrot.com/docs/sphinx/connectdrone.html
[wifi_freeflight]: https://www.parrot.com/us/support/products/anafi/how-prepare-your-anafi
[anafi_tools]: https://github.com/espetro/anafi_tools
[tello_tools]: https://github.com/espetro/tello_tools
[a_star]: https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2