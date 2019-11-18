#!/usr/bin/env python
#
# Copyright (c) 2019, Eurotec, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['wireless_monitor'],
    package_dir={'': 'src'}
)

setup(**d)
