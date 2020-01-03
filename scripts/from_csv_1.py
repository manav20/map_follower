#!/usr/bin/env python

import rospy
import numpy as np
from numpy import genfromtxt

mydata = genfromtxt('waypoint_file_x.csv', delimiter = '')

print(mydata[2])
