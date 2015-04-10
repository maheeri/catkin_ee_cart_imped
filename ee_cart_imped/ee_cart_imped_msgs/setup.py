#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	packages=['ee_cart_imped_msgs', 'ee_cart_imped_msgs.msg'],
	package_dir={'ee_cart_imped_msgs': 'src'},
	requires=['genpy', 'struct', 'ee_cart_imped_msgs', 'geometry_msgs', 'actionlib_msgs', 'std_msgs'],
	scripts=[]
)

setup(**d)
