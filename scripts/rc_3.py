#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import time
import numpy as np
import math
import serial
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point


console_serG = serial.Serial('/dev/ttyACM0',115200)
print('Start')
time.sleep(2)

class Car:
    global console_serG

    # Car Coordinates
    car_x = 0
    car_y = 0

    # IMU Initializations
    imu_del_t = 0.0067      # for 150 Hz
    imu_dx = 0
    imu_dy = 0
    imu_dz = 0
    imu_vx = 0
    imu_vy = 0
    imu_vz = 0
    imu_ax = 0
    imu_ay = 0
    imu_az = 0
    bias_ax = 0
    imu_ax_cor = 0

    # AMCL Initializations
    amcl_x = 0
    amcl_y = 0

    ## Callbacks
    def amclCallback(self,data):
        self.poseFromAmcl = data
	self.amcl_y = data.pose.pose.position.y
	self.amcl_x = data.pose.pose.position.x
        self.imu_dx = 0
        self.imu_dy = 0
        self.car_x = self.amcl_x
        self.car_y = self.amcl_y
        print('AMCL reading')

    def amclSubscriber(self):
	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amclCallback)

    def run_imu(self):
        done = False
        while done == False:
            char = console_serG.read()
            if char == 'I':
                self.imu_ax = console_serG.read(6)
                self.imu_ax = float(self.imu_ax)
                self.imu_ax = self.imu_ax/32768*2*9.8
                self.imu_ax_cor = self.imu_ax - self.bias_ax
                self.imu_vx = self.imu_vx + self.imu_ax_cor * self.imu_del_t
                self.imu_dx = self.imu_dx + self.imu_vx*self.imu_del_t + self.imu_ax*self.imu_del_t*self.imu_del_t*0.5
                #print('ax_cor:{} \t ax:{} \t dx:{}'.format(self.imu_ax_cor, self.imu_ax, self.imu_dx))
                #print('IMU reading'),
                done = True

    def run_imu_bias(self):
        print('Measuring Bias... '),
        itr = 1
        index = 0
        while itr < 600:
            char = console_serG.read()
            if char == 'I':
                self.imu_ax = console_serG.read(6)
                self.imu_ax = float(self.imu_ax)
                self.imu_ax = self.imu_ax/32768*2*9.8
                self.imu_ay = console_serG.read(6)
                self.imu_ay = float(self.imu_ay)
                self.imu_ay = self.imu_ay/32768*2*9.8
                self.imu_az = console_serG.read(6)
                self.imu_az = float(self.imu_az)
                self.imu_az = self.imu_az/32768*2*9.8
                self.bias_ax = self.bias_ax + self.imu_ax
                itr = itr + 1
                if itr%100 == 0:
                    print('.'),
        self.bias_ax = self.bias_ax/(itr-1)
        print('bias_ax = {} from {} readings'.format(self.bias_ax,itr))

    def amcl_N_imu(self):
        self.run_imu()
        self.car_x = self.amcl_x + self.imu_dx
        self.car_y = self.amcl_y + self.imu_dy

    def run(self):
        self.amcl_N_imu()
        print('The cordinates are {}\t{}'.format(self.car_x, self.car_y))


def main():
    # Node and Class initializations
    rospy.init_node('Car_on_Q4', anonymous=True)
    c = Car()
    # IMU Initialize 
    console_serG.write('IMU1')
    print('Initialiizing IMU...'),
    time.sleep(2)
    # Empty Buffer
    bl_no = 0
    console_serG.write('IMU1')
    while (bl_no < 20):
        char = console_serG.read()
        if char == '\n':
            bl_no = bl_no + 1
            print('.'),
    print('Buffer Emptied')
    # Measure Bias
    c.run_imu_bias()
    # Run at 150 Hz
    rate = rospy.Rate(150)
    c.amclSubscriber()
    while not rospy.is_shutdown():
        c.run()
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()



