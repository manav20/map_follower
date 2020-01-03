#!/usr/bin/env python

# COURSE: ECET 581
# Lab 3, Q1
# Name(1): Manav Wadhawan		PUID: 0030012262
# Name(2): Balaji
# Name(3): Oscar


import rospy
from std_msgs.msg import Float32
import time
import numpy as np
import math
# car hardware
import serial
from sensor_msgs.msg import LaserScan


console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)

class Lidar:
    global console_ser
    ## lidar parameters
    scan_number = 0
    angle_min = 0
    angle_max = 0
    scan_len = 0
    angle_increment = 0
    ranges = 0
    my_dist = 0.1
    my_dist_old = 0.1
    ## PID steer parameters : 
    dist_goal = 0.5                             # in meters
    err_old = 0
    err = 0
    err_sum = 0
    d_g = 0.5
    Kp = 40
    Kd = 10
    Ki = 0
    x_low = -20
    x_high = 20
    ## car hardware initializations
    y_vel = '+1000'                             # constant for y direction
    mode = 'A'                                  # constant for ackermann drive
    steer_angle = 0
    x_vel = 0
    x_vel_abs = 0
    x_vel_str = '0000'
    theta = 0
    ## Estimation parameters
    dist_est = 0
    dist_delta = 0
    dist_m = 0
    t1 = 0
    Kep = 1                                      # Estimation Scaling


    def scanCallback(self,data):
        self.my_dist_old = self.my_dist
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.ranges = data.ranges
        self.scan_number = self.scan_number + 1
        self.my_dist = self.ranges[180]
        self.t1 = rospy.get_time()

    def scanListener(self):
        rospy.Subscriber('scan', LaserScan, self.scanCallback)

    def run_est(self):
        Vy = 50
        Vx = Vy*np.sin(self.steer_angle*np.pi/180)
        Dx = Vx * (rospy.get_time() - self.t1)
        return Dx

    def run(self):
        self.dist_m = self.my_dist
        self.dist_delta = self.run_est()
        self.dist_m = self.dist_m - self.Kep*self.dist_delta
        self.err = self.dist_m - self.dist_goal
        x_cor = self.Kp*(self.err) + self.Kd*(self.err-self.err_old) + self.Ki*(self.err_sum)  # PID implementation
	if x_cor < self.x_low:                                      # bracket the error
		x_cor = self.x_low
	elif x_cor > self.x_high:
		x_cor = self.x_high
	self.x_vel = x_cor/(self.x_high) * 2048                     # scaling the error
	self.x_vel = int(self.x_vel)
        self.steer_angle = (self.x_vel/2048)
        if self.x_vel < 0 or self.x_vel == 0:
            direction = '-'
            self.x_vel_abs = abs(self.x_vel)
	elif self.x_vel > 0:
            direction = '+'
            self.x_vel_abs = abs(self.x_vel)
	self.y_vel = str(self.y_vel)
        self.x_vel_str = self.int_to_str()                          # self.x_vel to str
        cmd = self.mode + direction + self.x_vel_str + self.y_vel
        print("cmd : {} | dist from lidar =  {}m | self.dist_m = {}m | steer_angle  = {}deg | dist_delta = {} ".format(cmd, self.my_dist, self.dist_m, self.steer_angle, self.dist_delta))
	console_ser.write(cmd)                                      # write to serial
	self.err_old = self.err
        self.err_sum = self.err_sum + self.err

    def run_angle_calc(self):
        self.theta = np.arcsin(self.my_dist_old/self.my_dist)*180/np.pi
        if math.isnan(self.theta) == False:
            print(self.theta)


    def int_to_str(self):
        my_int = self.x_vel_abs
        if my_int > 999:
            my_int = str(my_int)
        elif (my_int < 999 or my_int == 999) and (my_int > 100 or my_int == 100):
            my_int = '0' + str(my_int)
        elif my_int < 100 and (my_int > 9 or my_int == 9):
            my_int = '00' + str(my_int)
        elif my_int < 9 and my_int > 0:
            my_int = '000' + str(my_int)
        elif my_int == 0:
            my_int ='0000'
        my_int = str(my_int)
        return my_int

    def run_tune(self):                     # rqt_plot /steer_cmd
        pub1 = rospy.Publisher('steer_cmd', Float32)
        pub1.publish(self.x_vel)
        pub2 = rospy.Publisher('steer_angle',Float32)
        pub2.publish(self.steer_angle)
        pub3 = rospy.Publisher('err', Float32)
        pub3.publish(self.err)
        pub4 = rospy.Publisher('dist_measure', Float32)
        pub4.publish(self.my_dist)
        pub5 = rospy.Publisher('dist_est', Float32)
        pub5.publish(self.dist_m)


def main():
    ## Node and class initializations
    rospy.init_node('rc_WallFollower', anonymous=True)
    l = Lidar()
    itr = 0
    done = False
    while not rospy.is_shutdown():
        l.scanListener()
        rate = rospy.Rate(150)
        while not rospy.is_shutdown():
            l.run()
            l.run_tune()
            rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
