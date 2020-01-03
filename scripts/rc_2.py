#!/usr/bin/env python

# COURSE: ECET 581
# Lab 3, Q5
# Name(1): Manav Wadhawan		PUID: 0030012262
# Name(2): Balaji
# Name(3): Oscar

## code status:
## untuned values of PID, P for fwd is too high
## controller frequency @150 hz
## 4 waypoints are choosen manually
### standard directions(stand behind the car): 	fwd is x
###				                left is +y
###				                right is -y
###				                anticlockwise yaw is +ve								


import rospy
from std_msgs.msg import Float32
import time
# rally car hardware
import serial
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


console_serG = serial.Serial('/dev/ttyACM0',115200)

class Car:
    global console_serG
    poseFromAmcl = PoseWithCovarianceStamped()

    # manually defined waypoints	
    waypoint_x = [9.22297477722,  9.5609331131,  9.82821846008, 10.0760278702,  10.3985872269,  10.8705215454,  11.1512451172,  11.4630308151,  11.7872953415, 12.0715732574,  12.5092477798,  12.9083614349, 12.959431648, 13.0315103531,  13.0642805099,  13.0588378906,  13.0395879745]
    waypoint_y = [0.0268885232508, 0.00712374784052, -0.00839765090495, -0.0787715613842,-0.046705879271, -0.085138015449, -0.136199742556, -0.169776380062,-0.0992393344641,-0.135954737663, -0.112006239593,  0.361669361591, 0.636179447174,0.964450240135,1.27673280239,1.62345945835,1.95718026161]

    my_x = 0
    my_y = 0
    waypoint_index = 0

    ##
    run_itr = 0

    ## PID_steer oy y parameters : 
    dist_goal = 0.0
    err_old1 = 0
    err1 = 0
    err_sum1 = 0
    d_g = 0.0
    Kp1 = 25
    Kd1 = 1
    Ki1 = 0
    y_low = -10
    y_high = 10

    ## PID_fwd or x parameters
    dist_goal = 0.0
    err_old2 = 0
    err2 = 0
    err_sum2 = 0
    d_g = 0.0
    Kp2 = 10
    Kd2 = 0
    Ki2 = 0
    x_low = -20
    x_high = 20

    ## car hardware initializations
    mode = 'A'

    ## imu initializations
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
    bias_ax = 0

    ## control the heading; y(or steer) then x(or fwd)
    def run(self):
        val_x = 0
        val_y = 0
        waypoint_number = 9

        # choose val_x and val_y as per waypoint index
        val_x = self.waypoint_x[self.waypoint_index]
        val_y = self.waypoint_y[self.waypoint_index]

        # log iterations
        self.run_itr = self.run_itr + 1
        flag = True
	print('before IMU')
        # use imu to uypdate my_x and my_y
	console_serG.write('IMU1')
        while not rospy.is_shutdown() and flag == True:
            char = console_serG.read()
            print('reading char')
            if char == 'I':
                print('inside if')
                self.val_ax = console_serG.read(6)
                self.val_ax = float(self.val_ax)
                self.val_ax = self.val_ax/32768*2*9.8
                self.val_ax_cor = self.val_ax - self.bias_ax
                self.val_vx = self.val_vx + self.val_ax_cor * self.del_t
                self.val_dx = self.val_dx + self.val_vx*self.del_t + self.val_ax*self.del_t*self.del_t*0.5
                print(' ax_cor:{} \t ax:{} \t dx:{}'.format(self.val_ax_cor, self.val_ax, self.val_dx))
                flag = False
        print('imu complete')
        self.my_x = self.val_dx
        self.err1 = val_y - self.my_y
	y_cor = self.Kp1*(self.err1) + self.Kd1*(self.err1 - self.err_old1) + self.Ki1*(self.err_sum1)
        # bracket the y_cor or steer
	if y_cor < self.y_low:
            y_cor = self.y_low
	elif y_cor > self.y_high:
            y_cor = self.y_high
        # scale to -2048 to 2048
	self.steer = y_cor/self.y_high * 2048
        self.steer = int(self.steer)
	# note the steer_direction for string formingw
	if self.steer < 0 or self.steer == 0:
            steer_direction = '+'
            self.steer_abs = abs(self.steer)
	elif self.steer > 0:
            steer_direction = '-'
	self.steer_abs = abs(self.steer)
        self.steer = self.int_to_str_steer(self.steer_abs)
        self.err_sum1 = self.err1 + self.err_sum1
        self.err_old1 = self.err1

        self.err2 = val_x - self.my_x
	x_cor = self.Kp2*(self.err2) + self.Kd2*(self.err2 - self.err_old2) + self.Ki2*(self.err_sum2)
        # bracket the x_cor or fwd
        if x_cor < self.x_low:
            x_cor = self.x_low
        elif x_cor > self.x_high:
            x_cor = self.x_high
        # scale to -2048 to 2048
        self.fwd = x_cor/self.x_high * 2048
        self.fwd = int(self.fwd)
        # note the direction for string forming
        if self.fwd < 0 or self.fwd == 0:
            fwd_direction = '-'
            self.fwd_abs = abs(self.fwd)    # fwd values cannot be negative
            self.fwd_abs = 0
        elif self.steer > 0:
            fwd_direction = '+'
            self.fwd_abs = abs(self.fwd)
        if self.fwd_abs < 300 and self.fwd != 0:    # drive motor dead 
            self.fwd_abs = 300
        self.fwd = self.int_to_str_fwd(self.fwd_abs)
	self.err_sum2 = self.err2 + self.err_sum2
	self.err_old2 = self.err2

        cmd = self.mode + steer_direction + self.steer + fwd_direction + self.fwd
        print("cmd:{}='A+s+f'| /pose x = {}, y = {} || wp_no: {}, itr: {}".format(cmd,self.my_x, self.my_y, self.waypoint_index,self.run_itr))
        console_serG.write(cmd)
        print('command writing complete')

    ## This function takkes an integer and converts it into a string. It also adds leading zeros to make sure the string is in 4 digit format. The number entered in this function must be >=0.
    def int_to_str_steer(self, steer_abs):
        my_int = self.steer_abs
        if my_int > 999:
            my_int = str(my_int)
        elif (my_int < 999 or my_int == 999)  and (my_int > 100 or my_int == 100):
            my_int = '0' + str(my_int)
        elif my_int < 100 and my_int > 9:
            my_int = '00' + str(my_int)
        elif my_int < 9 and my_int > 0:
            my_int = '000' + str(my_int)
        elif my_int == 0:
            my_int = '0000'
	my_int = str(my_int)
	return my_int

    def int_to_str_fwd(self, fwd_abs):
        my_int = self.fwd_abs
	if my_int > 999:
            my_int = str(my_int)
	elif (my_int < 999 or my_int == 999)  and (my_int > 100 or my_int == 100):
            my_int = '0' + str(my_int)
	elif my_int < 100 and my_int > 9:
            my_int = '00' + str(my_int)
	elif my_int < 9 and my_int > 0:
            my_int = '000' + str(my_int)
	elif my_int == 0:
            my_int = '0000'
	my_int = str(my_int)
	return my_int


def main():
    ## Node and class initializations
    rospy.init_node('Car_on_Q4', anonymous=True)

    ## Initialize IMU
    console_serG.write('IMU1')
    print('Initializing IMU')
    time.sleep(2)
    bl_no = 0
    console_serG.write('IMU1')
    while (bl_no < 20):    # leave 20 lines
        char = console_serG.read()
        if char == '\n':
            bl_no = bl_no + 1
            print('.'),
    print('\n'+'Buffer emptied')

    c = Car()
    main_itr = 0
    done = False
    print("Initializing ...")
    time.sleep(3)
    while not rospy.is_shutdown():
        rate = rospy.Rate(150)
        print('in')
        while not rospy.is_shutdown():
            if c.run_itr%15==0:         # 10hz waypoint update
                c.waypoint_index = c.waypoint_index + 1
                print("the number is : {}".format(c.waypoint_index))
            print('before run')
            c.run()
            print('after run')
            rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
