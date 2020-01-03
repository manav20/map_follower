#!/usr/bin/env python


## Code to read IMU readings
    # sorting in ax, ay, az done
    # converting to int
    # remove ax bias

import rospy
from std_msgs.msg import Float32
import time
import numpy as np
import math
import serial

class Imu():

    del_t = 0.0067              # for 150 Hz
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
    val_theta_z = 0
    bias_ax = 0
    bias_gz = 0
    reading = ''


    def run_bias():
        time.sleep(2)
        bl_no = 0
        console_ser.write('IMU2')
        while (bl_no < 20):         # leave 20 lines
            char = console_ser.read()
            if char == '\n':
                bl_no = bl_no + 1
            print('.'),
        print('\n'+'Buffer emptied')
        print('Measuring Bias')
        itr = 1
        all_val = np.array([])
        index = 0
        while not rospy.is_shutdown() and itr < 500:
            char = console_ser.read()
            if char == 'I':
                self.val_ax = console_ser.read(6)
                self.val_ax = float(val_ax)
                self.val_ax = self.val_ax/32768*2*9.8

                self.val_ay = console_ser.read(6)
                self.val_ay = float(val_ay)
                self.val_ay = self.val_ay/32768*2*9.8

                self.val_az = console_ser.read(6)
                self.val_az = float(self.val_az)
                self.val_az = self.val_az/32768*2*9.8

                self.val_gz = console_ser.read(6)
                self.val_gz = float(self.val_gz)
                self.val_gz = self.val_gz/32768*250

                self.bias_ax = self.bias_ax + self.val_ax
                self.bias_gz = self.bias_gz + self.val_gz
                itr = itr + 1
                if itr%50 == 0:
                    print('.'),
        self.bias_ax = self.bias_ax/itr
        self.bias_gz = float(self.bias_gz/itr)
        print('\nBias_ax = {}, Bias_gz = {}'.format(self.bias_ax, self.bias_gz))



def main():
    rospy.init_node('rc_IMU')
    console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
    console_ser.write('IMU2')
    pub = rospy.Publisher('imu_publish', Float32, queue_size = 10)
    time.sleep(3)
    #rate = rospy.Rate(150)
    while not rospy.is_shutdown():
        char = console_ser.read()
        if char == 'I':
            val_ax = console_ser.read(6)
            val_ax = float(val_ax)
            val_ax_raw = val_ax
            val_ax = val_ax/32768*2*9.8
            val_ax_cor = val_ax - bias_ax
            val_vx = val_vx + val_ax_cor * del_t
            val_dx = val_dx + val_vx*del_t + val_ax*del_t*del_t*0.5
            val_gz = console_ser.read(6)
            val_gz = float(val_gz)
            val_gz = val_gz/32768*250
            val_gz_cor = val_gz - bias_gz
            val_theta_z = val_theta_z + del_t*(val_gz_cor)
            #print(' ax_cor:{} \t ax:{} \t dx:{}m '.format(val_ax_cor, val_ax, val_dx))
            print(' gz_cor:{} | gz:{} | theta_z:{}deg'.format(val_gz_cor, val_gz, val_theta_z))
            pub.publish(val_theta_z)
    rospy.spin()

if __name__ == '__main__':
    main()



































