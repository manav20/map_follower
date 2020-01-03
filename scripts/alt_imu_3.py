#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import time
import numpy as np
import math
import serial


def main():
    rospy.init_node('rc_IMU')
    console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
    #readBuf  = console_ser.read(20)
    #console_ser.write('IMU1')
    while(console_ser.inWaiting() > 0):
        readBuf = console_ser.read(1)
        print(readBuf)
        console_ser.write('IMU1')
   	#console_ser.write('IMU1')
    rospy.spin()
    

if __name__ == '__main__':
    main()
