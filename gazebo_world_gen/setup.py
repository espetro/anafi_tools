#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
from __future__ import print_function
from setuptools import setup
from os import read

setup(
    name = "gazebo_world_gen",
    version = "0.0.1",
    author = "Joaquin Terrasa Moya",
    author_email = "quino.terrasa+dev@gmail.com",
    description = "A gazebo world generator",
    license = "BSD 3-Clause Revised",
    keywords = "gazebo python numpy scipy",
    url = "https://github.com/espetro/anafi_tools",
    packages=['generators'],
    include_package_data=True,
    install_requires = [
        "numpy==1.11.0"
    ],
    zip_safe=False,
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Topic :: Software Development",
        "License :: OSI Approved :: BSD License",
    ],
)