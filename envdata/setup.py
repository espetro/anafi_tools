#!/usr/bin/python3.5
# -*- coding: utf-8 -*-
from __future__ import print_function
from setuptools import setup

import sys

try:
    import olympe
except ImportError:
    sys.exit("Sorry, you are not using Parrot Olympe Python 3.5 runtime. Please load it before installing the package.")

setup(
    name = "envdata",
    version = "0.0.1",
    author = "Joaquin Terrasa Moya",
    author_email = "quino.terrasa+dev@gmail.com",
    description = "A package to stream, manipulate and process sensor data from OptiTrack and Parrot Telemetryd",
    license = "BSD 3-Clause Revised",
    keywords = "gazebo python numpy scipy",
    url = "https://github.com/espetro/anafi_tools",
    packages=['telemetry', 'optitrack'],
    include_package_data=True,
    install_requires = [
        #rclpy is required but can't be installed through pypi
        "numpy>=1.16.0",
        "pandas>=0.24.1"

    ],
    zip_safe=False,
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Topic :: Software Development",
        "License :: OSI Approved :: BSD License",
    ],
)