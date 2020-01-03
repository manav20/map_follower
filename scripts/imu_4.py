#!/usr/bin/env python


## Code to read IMU readings
    # sorting in ax, ay, az done
    # converting to int
    # remove ax bias, done
    # gyro added

import rospy
from std_msgs.msg import Float32
import time
import numpy as np
import math
import serial


def main():
    rospy.init_node('rc_IMU')
    console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
    console_ser.write('IMU2')
    print('Initializing IMU')
    time.sleep(2)
    bl_no = 0
    console_ser.write('IMU2')
    while (bl_no < 20):    # leave 20 lines
        char = console_ser.read()
        if char == '\n':
            bl_no = bl_no + 1
            print('.'),
    print('\n'+'Buffer emptied')

    del_t = 0.0067      # for 150 Hz
    val_dx = 0
    val_dy = 0
    val_dz = 0
    val_vx = 0
    val_vy = 0
    val_vz = 0
    val_ax = 0
    val_ay = 0
    val_az = 0
    val_gz = 0
    val_thetaz = 0
    bias_ax = 0
    bias_ay = 0
    bias_az = 0
    bias_gz = 0
    reading = ''

## run car for test
    #console_ser.write('A+0000+1000')

    print('Measuring Bias')
    itr = 1
    bias_ax = 0
    all_val = np.array([])
    index = 0
    while not rospy.is_shutdown() and itr < 300:
        char = console_ser.read()
        if char == 'I':
            val_ax = console_ser.read(6)
            val_ax = float(val_ax)
            val_ax = val_ax/32768*2*9.8
            val_ay = console_ser.read(6)
            val_ay = float(val_ay)
            val_ay = val_ay/32768*2*9.8
            val_az = console_ser.read(6)
            val_az = float(val_az)
            val_az = val_az/32768*2*9.8
            val_gz = console_ser.read(6)
            val_gz = float(val_gz)
            val_gz = val_gz/32768*250
            itr = itr + 1
            bias_ax = bias_ax + val_ax
            bias_ay = bias_ay + val_ay
            bias_az = bias_az + val_az
            bias_gz = bias_gz + val_gz
            index = index + 1
            if itr%50 == 0:
                print('.'),
    bias_ax = bias_ax/itr
    bias_ay = bias_ay/itr
    bias_az = bias_az/itr
    bias_gz = bias_gz/itr
    print('\nbias_ax = {}m/s2'.format(bias_ax))
    print('bias_ay = {}m/s2'.format(bias_ay))
    print('bias_az = {}m/s2'.format(bias_az))
    print('bias_gz = {}deg/s'.format(bias_gz))
    print('Measuring Filter Parameters')
    itr = 0
    while not rospy.is_shutdown() and itr < 300:
        char = console_ser.read()
        if char == 'I':
            val_ax = console_ser.read(6)
            val_ax = float(val_ax)
            val_ax = val_ax/32768*2*9.8
            val_ax_cor = val_ax - bias_ax
            val_ay = console_ser.read(6)
            val_ay = float(val_ay)
            val_ay = val_ay/32768*2*9.8
            val_az = console_ser.read(6)
            val_az = float(val_az)
            val_az = val_az/32768*2*9.8
            itr = itr + 1
            all_val = np.append(all_val, val_ax_cor)
    all_val_std = all_val.std()
    print('Standard Deviation : {}'.format(all_val_std))
    Q = all_val_std * all_val_std
    print('Covariance : {}'.format(Q))

    while not rospy.is_shutdown():
        char = console_ser.read()
        if char == 'I':
            val_ax = console_ser.read(6)
            val_ax = float(val_ax)
            val_ax = val_ax/32768*2*9.8
            val_ax_cor = val_ax - bias_ax
            val_vx = val_vx + val_ax_cor * del_t
            val_dx = val_dx + val_vx*del_t + val_ax*del_t*del_t*0.5

            val_ay = console_ser.read(6)
            val_ay = float(val_ax)
            val_ay = val_ay/32768*2*9.8
            val_ay_cor = val_ay - bias_ay
            val_vy = val_vy + val_ay_cor * del_t
            val_dy = val_dy + val_vy*del_t + val_ay*del_t*del_t*0.5

            val_gz = console_ser.read(6)
            val_gz = float(val_gz)
            val_gz = val_gz/32768*250
            val_gz_cor = val_gz - bias_gz
            val_thetaz = val_thetaz + val_gz_cor * del_t

            print(' ax_cor:{} | ax:{} | ay_cor:{} | ay:{} | gz_cor:{} | gz:{}'.format(val_ax_cor, val_ax, val_ay_cor, val_ay, val_gz_cor, val_gz))
    rospy.spin()

if __name__ == '__main__':
    main()



































