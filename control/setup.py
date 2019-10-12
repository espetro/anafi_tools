#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
from __future__ import print_function
from setuptools import setup

setup(
    name = "control_parrot_drones",
    version = "0.0.1",
    author = "Joaquin Terrasa Moya",
    author_email = "quino.terrasa+dev@gmail.com",
    description = "A package to easily control Parrot Bebop2 and Anafi drones",
    license = "BSD 3-Clause Revised",
    keywords = "gazebo python numpy scipy",
    url = "https://github.com/espetro/anafi_tools",
    packages=['anafi', 'bebop', 'ros1', 'parrot-olympe'],
    include_package_data=True,
    install_requires = [],
    zip_safe=False,
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Topic :: Software Development",
        "License :: OSI Approved :: BSD License",
    ],
)