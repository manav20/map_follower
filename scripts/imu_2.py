#!/usr/bin/env python


## Code to read IMU readings
    # soriutng in ax, ay, az done
    # converting to int


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

    val_dx = 0
    val_dy = 0
    val_dz = 0
    val_vx = 0
    val_vy = 0
    val_vz = 0
    val_ax = 0
    val_ay = 0
    val_az = 0
    reading = '\n'
    while not rospy.is_shutdown():
        reading = reading + char
        if char == 'U':         # new reading
            print(reading)
            print(reading[0] + reading[1] + reading[2])
            val_ax = reading[3:9]
            val_ax = float(val_ax)
            val_ax = val_ax/32768*2
            val_ay = reading[9:15]
            val_ay = float(val_ay)
            val_ay = val_ay/32768*2
            val_az = reading[15:21]
            val_az = float(val_az)
            val_az = val_az/32768*2
            reading = ''
            print(' Ac => ax:{} ay:{} az:{}'.format(val_ax, val_ay, val_az))
            val_vx = val_vx + val_ax*0.0067
            val_vy = val_vy + val_ay*0.0067
            val_vz = val_vz + val_az*0.0067
            print('Vel => vx:{} vy:{} vz:{}'.format(val_vx, val_vy, val_vz))
            val_dx = val_dx + val_vx*0.0067 + 0.5*val_ax*0.0067*0.0067
            val_dy = val_dy + val_vy*0.0067 + 0.5*val_ay*0.0067*0.0067
            val_dz = val_dz + val_vz*0.0067 + 0.5*val_az*0.0067*0.0067
            print(' Dx => dx:{} dy:{} dz:{}'.format(val_dx, val_dy, val_dz))
            char = ''
        char = console_ser.read()
    rospy.spin()


if __name__ == '__main__':
    main()



































