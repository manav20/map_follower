#!/usr/bin/env python


## Code to read IMU readings

import rospy
from std_msgs.msg import Float32
import time
import numpy as np
import math
import serial


def main():
    rospy.init_node('rc_IMU')
    console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
    console_ser.write('IMU1')
    print('Initializing IMU')
    time.sleep(2)
    bl_no = 0
    console_ser.write('IMU1')
    while (bl_no < 20):    # leave 20 lines
        char = console_ser.read()
        if char == '\n':
            bl_no = bl_no + 1
            print('.'),
    print('\n'+'Buffer emptied')

    reading = ''
    while not rospy.is_shutdown():
        char = console_ser.read()
        reading = reading + char
        if char == 'U':
            print(reading)
            reading = ''
    rospy.spin()


if __name__ == '__main__':
    main()
