#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
from __future__ import print_function
from setuptools import setup

setup(
    name = "envdata",
    version = "0.0.1",
    author = "Joaquin Terrasa Moya",
    author_email = "quino.terrasa+dev@gmail.com",
    description = "A package to easily manipulate streamed data from sensors",
    license = "BSD 3-Clause Revised",
    keywords = "gazebo python numpy scipy",
    url = "https://github.com/espetro/anafi_tools",
    packages=['ros2', 'ros1', 'parrot-olympe', 'optitrack', 'parrot-sphinx'],
    include_package_data=True,
    install_requires = [],
    zip_safe=False,
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Topic :: Software Development",
        "License :: OSI Approved :: BSD License",
    ],
)