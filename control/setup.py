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
    name = "control_parrot_drones",
    version = "0.0.1",
    author = "Joaquin Terrasa Moya",
    author_email = "quino.terrasa+dev@gmail.com",
    description = "A package to easily control Parrot Bebop2 and Anafi drones",
    license = "BSD 3-Clause Revised",
    keywords = "gazebo python numpy scipy",
    url = "https://github.com/espetro/anafi_tools",
    packages=['drone', 'teleop'],
    include_package_data=True,
    install_requires = [
        "pygame>=1.9.6",  # olympe is also required but can't be installed via pypi/wheels
    ],
    zip_safe=False,
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Topic :: Software Development",
        "License :: OSI Approved :: BSD License",
    ],
)