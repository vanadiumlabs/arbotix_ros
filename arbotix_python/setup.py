#!/usr/bin/env python3

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['bin/arbotix_gui', 'bin/arbotix_terminal'],
    packages=['arbotix_python'],
    package_dir={'': 'src'},
    )

setup(**d)
