#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['sroscore'],
    package_dir={'': 'src'},
    scripts=['scripts/sroscore'],
    requires=[]
)
#requires=['genmsg', 'genpy', 'roslib', 'rospkg']

setup(**d)