#!/usr/bin/env python2.7

import rospy
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
import math
import serial
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion

## Golbal Variables
console_serG = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(2)
print('Start')

class Car:
    global console_serG

    # Map Initilizations
    waypoint_dx = []
    waypoint_dy = []
    waypoint_yaw = []
    waypoint_index = 0
    waypoint_rate = 150 #155         #120

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
    imu_gz = 0
    imu_yaw = 0
    imu_bias_ax = 0
    imu_bias_ay = 0
    imu_bias_az = 0
    imu_bias_gz = 0
    imu_ax_cor = 0
    imu_ay_cor = 0
    imu_az_cor = 0
    imu_gz_cor = 0

    # AMCL Initializations
    amcl_dx_1 = 0
    amcl_dy_1 = 0
    amcl_dx_2 = 0
    amcl_dy_2 = 0
    amcl_vel_x = 0
    amcl_vel_y = 0
    amcl_t1 = 0.1
    amcl_t2 = 0
    amcl_detT = 0
    amcl_yaw = 0
    amcl_cov_x = 0
    amcl_cov_y = 0
    amcl_cov_yaw = 0

    # Model Initialization
    pred_dx = 0
    pred_vx = 0
    pred_dy = 0
    pred_vy = 0
    pred_yaw = 0

    # Car Initilizations
    car_bias_steer = 240
    car_bias_dx = 0
    car_x = 0
    car_y = 0
    car_yaw = 0
    car_vel_x = 0

    # PID Initializations
    pid_dy_goal = 0.0
    pid_dy_err_old = 0
    pid_dy_err = 0
    pid_dy_err_sum = 0
    pid_dy_err_all = 0
    pid_dy_Kp = 3.7  #4.2
    pid_dy_Kd = 34 #70      #34
    pid_dy_Ki = 0
    pid_dy_err_low = -10
    pid_dy_err_high = 10
    pid_dy_cmd = 0
    pid_yaw = 0

    pid_dx_goal = 0.0
    pid_dx_err_old = 0
    pid_dx_err = 0
    pid_dx_err_sum = 0
    pid_dx_err_all = []
    pid_dx_Kp = 7      #6.7         #7
    pid_dx_Kd = 20
    pid_dx_Ki = 0
    pid_dx_err_low = 0
    pid_dx_err_high = 40
    pid_dx_cmd = 0
    pid_x_vel = 0
    deltaT = 0.0067


    # RViz Initializations
    rviz_marker_id = 1
    rviz_publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 10)
    rviz_path = Marker()
    rviz_car_marker_id = 0

    # Misc.
    counter = 0

#    def run_imu_bias(self):
#        console_serG.write('IMU0')
#        time.sleep(2)
#        console_serG.write('IMU2')
#        time.sleep(2)
#        bl_no = 0
#        console_serG.write('IMU2')
#        while (bl_no < 20):
#            char = console_serG.read()
#           if char == '\n':
#                bl_no = bl_no + 1
#        print('-Buffer Emptied')
#        print('-Measuring Bias')
#        itr = 1
#        while not rospy.is_shutdown() and itr < 300:
#            char = cosole_serG.read()
#            if char == 'I':
#                self.iimu_ax = console_serG.read(6)
#                self.imu_ax = float(self.imu_ax)
#                self.imu_ax = self.imu_ax/32768*2*9.8
#                self.imu_ay = console_serG.read(6)
#                self.imu_ay = float(self.imu_ay)
#                self.imu_ay = self.imu_ay/32768*2*9.8
#                self.imu_az = console_serG.read(6)
#                self.imu_az = float(self.imu_az)
#                self.imu_az = self.imu_az/32768*2*9.8
#                self.imu_gz = console_serG.read(6)
#                self.imu_gz = float(self.imu_gz)
#                self.imu_gz = self.imu_gz/32768*250
#                self.imu_bias_ax = self.imu_bias_ax + self.imu_ax
#                self.imu_bias_ay = self.imu_bias_ay + self.imu_ay
#                self.imu_bias_az = self.imu_bias_az + self.imu_az
#                self.imu_bias_gz = self.imu_bias_gz + self.imu_gz
#                itr = itr + 1
#        self.imu_bias_ax = self.imu_bias_ax/itr
#        self.imu_bias_ay = self.imu_bias_ay/itr
#        self.imu_bias_az = self.imu_bias_az/itr
#        self.imu_bias_gz = self.imu_bias_gz/itr
#        print('-imu_bias_ax = {} m/s2'.format(self.imu_bias_ax))
#        print('-imu_bias_ay = {} m/s2'.format(self.imu_bias_ay))
#       print('-imu_bias_az = {} m/s2'.format(self.imu_bias_az))
#        print('-imu_bias_gz = {} deg/s'.format(self.imu_bias_gz))


    def display_path(self):
        while self.rviz_marker_id < self.waypoint_dx.size and not rospy.is_shutdown():
            points = Marker()
            points.header.frame_id = "/map"      # publish self.path in map frame		
            points.type = points.POINTS
            points.action = points.ADD
            points.lifetime = rospy.Duration(0)
            points.id = self.rviz_marker_id
            points.scale.x = 0.1
            points.scale.y = 0.1
            points.color.a = 1.0
            points.color.r = 0.0
            points.color.g = 0.0
            points.color.b = 1.0
            points.pose.orientation.w = 1.0
            point = Point()
            point.x = self.waypoint_dx[self.rviz_marker_id]
            point.y = self.waypoint_dy[self.rviz_marker_id]
            points.points.append(point);
            self.rviz_publisher.publish(points)

            self.rviz_path.header.frame_id = "/map"	# publish path in map frame
            self.rviz_path.type = self.rviz_path.LINE_STRIP
            self.rviz_path.action = self.rviz_path.ADD
            self.rviz_path.lifetime = rospy.Duration(0)
            self.rviz_path.id = 0
            self.rviz_path.scale.x = 0.05
            self.rviz_path.color.a = 1.0
            self.rviz_path.color.r = 0.0
            self.rviz_path.color.g = 1.0
            self.rviz_path.color.b = 0.0
            self.rviz_path.pose.orientation.w = 1.0
            self.rviz_path.points.append(point);
            self.rviz_publisher.publish(self.rviz_path)
            self.rviz_marker_id = self.rviz_marker_id + 1

    def run_display(self, val1, val2, c1,c2, c3, s):
        self.rviz_car_marker_id = self.rviz_car_marker_id + 1
        points = Marker()
        points.header.frame_id = "/map"      # publish self.path in map frame		
        points.type = points.POINTS
        points.action = points.ADD
        points.lifetime = rospy.Duration(0)
        points.id = self.rviz_car_marker_id
        points.scale.x = s
        points.scale.y = s
        points.color.a = 1.0
        points.color.r = c1
        points.color.g = c2
        points.color.b = c3
        points.pose.orientation.w = 1.0
        point = Point()
        point.x = val1
        point.y = val2
        points.points.append(point);
        self.rviz_publisher.publish(points)

    def run_sensor_fusion(self,val1,val2):
        self.car_x = val1
        self.car_y = val2
        self.car_vel_x = self.amcl_vel_x
        self.car_yaw = self.amcl_yaw

    def amclCallback(self, data):
        self.amcl_t1 = rospy.get_time()
        self.amcl_dx_1 = data.pose.pose.position.x
        self.amcl_dy_1 = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        q_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        euler = euler_from_quaternion(q_list)
        self.amcl_yaw = euler[2]
        self.amcl_delT = self.amcl_t1 - self.amcl_t2
        self.amcl_vel_x = ( self.amcl_dx_1 - self.amcl_dx_2)/(self.amcl_t1 - self.amcl_t2)
        self.run_sensor_fusion(self.amcl_dx_1, self.amcl_dy_1)
        self.amcl_dx_2 = self.amcl_dx_1
        self.amcl_dy_2 = self.amcl_dy_1
        self.amcl_t2 = self.amcl_t1
        self.amcl_cov_x = data.pose.covariance[0]
        self.amcl_cov_y = data.pose.covariance[7]
        self.amcl_cov_yaw = data.pose.covariance[35]

    def amclSubscriber(self):
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amclCallback)

    def run_transform(self):
        theta = self.car_yaw
        tx = -self.car_x
        ty = -self.car_y
        R = np.matrix([   [math.cos(theta), math.sin(theta), 0],
                          [-math.sin(theta), math.cos(theta), 0],
                          [0, 0, 1]     ])
        T = np.matrix([   [1, 0, tx],
                          [0, 1, ty],
                          [0, 0,  1]    ])
        map_pose = np.matrix([[self.waypoint_dx[self.waypoint_index]],
                              [self.waypoint_dy[self.waypoint_index]],
                              [1]])
        [wp_dx, wp_dy, one] =  np.dot(np.dot(R, T),map_pose)
        return(wp_dx, wp_dy)

    def run(self):
        wp_dx, wp_dy = self.run_transform()
        print('Orginal : {}, {} | @{} | Trnsformed :{}, {}'.format(self.waypoint_dx[self.waypoint_index], self.waypoint_dy[self.waypoint_index], self.waypoint_index, wp_dx, wp_dy))
        #self.pid_dx_err = self.waypoint_dx[self.waypoint_index] - self.car_x
        self.pid_dx_err = wp_dx
        #print(self.waypoint_dx, self.car_x)
        if self.pid_dx_err < self.pid_dx_err_low:
            self.pid_dx_err = self.pid_dx_err_low
        elif self.pid_dx_err > self.pid_dx_err_high:
            self.pid_dx_err = self.pid_dx_err_high
        #print('The err = {}'.format(self.pid_dx_err))
        self.pid_dx_cmd = self.pid_dx_Kp*(self.pid_dx_err) + self.pid_dx_Kd*(self.pid_dx_err- self.pid_dx_err_old)
        #print('after PID = {}'.format(self.pid_dx_cmd))
        self.pid_dx_cmd = int((self.pid_dx_cmd/self.pid_dx_err_high)*2048)
        if self.pid_dx_cmd > 2048:
            self.pid_dx_cmd = 2048
        self.pid_dx_err_old = self.pid_dx_err

        #self.pid_dy_err = - self.waypoint_dy[self.waypoint_index] + self.car_y
        self.pid_dy_err = - wp_dy
        #print(self.pid_dy_err)
        self.pid_dy_cmd = self.pid_dy_Kp*(self.pid_dy_err) + self.pid_dy_Kd*(self.pid_dy_err- self.pid_dy_err_old)
        #print(self.pid_dy_cmd)
        if self.pid_dy_cmd < self.pid_dy_err_low:
            self.pid_dy_cmd = self.pid_dy_err_low
        elif self.pid_dy_cmd > self.pid_dy_err_high:
            self.pid_dy_cmd = self.pid_dy_err_high
        self.pid_dy_cmd = int((self.pid_dy_cmd/self.pid_dy_err_high)*2048)
        self.pid_dy_cmd = self.pid_dy_cmd + self.car_bias_steer
        if self.pid_dy_cmd < -2048:
            self.pid_dy_cmd = -2048
        elif self.pid_dy_cmd > 2048:
            self.pid_dy_cmd = 2048
        self.pid_dy_err_old = self.pid_dy_err
        cmd = 'A' + self.int_to_str(self.pid_dy_cmd) + self.int_to_str(self.pid_dx_cmd)
        console_serG.write(cmd)
        #print('Err X : {} | Err Y : {}'.format(self.pid_dx_err, self.pid_dy_err))
        #print('cmd:{} | wp: {}, {} | @ {} | car_x,y:{},{} '.format(cmd, self.waypoint_dx[self.waypoint_index], self.waypoint_dy[self.waypoint_index], self.waypoint_index, self.car_x, self.car_y))
        #print('Cov ==  X : {} Y: {} Yaw:{}'.format(self.amcl_cov_x, self.amcl_cov_y, self.amcl_cov_yaw))
        self.run_estimate()
        self.run_display(self.car_x, self.car_y, 0.0, 0.0, 0.0, 0.1)
        #self.run_display(self.waypoint_dx[self.waypoint_index], self.waypoint_dy[self.waypoint_index], 1.0, 1.0, 0.0, 0.2)

    def convert_to_val(self, throttle, steer):
        speed = float(throttle)
        speed = -0.0000023727*speed*speed + 0.0076432741*speed - 1.2083497298
        #speed = speed/256
        steering_angle = float(steer)
        steering_angle = steering_angle/2048*50
        steering_angle = steering_angle*math.pi/180
        #print('cmd_throttle : {} | Speed: {} | cmd_angle: {} | steering_angle :{}'.format(throttle, speed, steer, steering_angle))
        return (speed, steering_angle)

    def run_estimate(self):
        self.pid_x_vel, steer = self.convert_to_val(self.pid_dx_cmd, self.pid_dy_cmd)
        self.car_x = self.car_x + self.deltaT*(self.pid_x_vel * math.sin(steer))
        self.car_y = self.car_y + self.deltaT*(self.pid_x_vel * math.cos(steer))
        #self.car_yaw = steer

    def int_to_str(self,val):
        flag = False
        if val < 0:
            flag = True
        val = abs(val)
        if val > 999:
            val = str(val)
        elif (val < 999 or val == 999) and (val > 100 or val == 100):
            val = '0' + str(val)
        elif (val < 100 and val > 9):
            val = '00' + str(val)
        elif (val < 9 or val==9)  and val > 0:
            val = '000' + str(val)
        elif val == 0:
            val = '0000'
        if flag == True:
            val = '-' + str(val)
        elif flag == False:
            val = '+' + str(val)
        return(val)

    def run_stop(self):
        print('\nCar Stopped')
        console_serG.write('A+0000+0000')
        self.car_vel_x = 0

    def run_tune(self, count):                     # rqt_plot /steer_cmd
        if count % self.waypoint_rate == 0:
            self.pid_dx_err_all = np.append(self.pid_dx_err_all, -3)
            self.pid_dy_err_all = np.append(self.pid_dy_err_all, -120)
        else:
            self.pid_dx_err_all = np.append(self.pid_dx_err_all, self.pid_dx_err)
            self.pid_dy_err_all = np.append(self.pid_dy_err_all, self.pid_dy_err)
        if self.waypoint_index == self.waypoint_dx.size:
            index1 = []
            index2 = []
            self.run_stop()
            for i in range(len(self.pid_dx_err_all)):
                index1 = np.append(index1, i)
            for i in range(len(self.pid_dy_err_all)):
                index2 = np.append(index2, i)
            plt.subplot(211)
            plt.plot(index1, self.pid_dx_err_all)
            plt.title('Error X vs Index')
            plt.subplot(212)
            plt.plot(index2, self.pid_dy_err_all)
            plt.title('Error Y vs Index')
            plt.show()
            plt.close()

    def car_shutdown(self):
        print('\nShutdown...\n')
        console_serG.write('A+0000+0000')


def main():
    rospy.init_node('Race')
    c = Car()
    rospy.on_shutdown(c.car_shutdown)
    print('Initializing Waypoints on the Map')
    c.waypoint_dx = genfromtxt('waypoint_file_race_x.csv',delimiter = '')
    c.waypoint_dy = genfromtxt('waypoint_file_race_y.csv', delimiter = '')
    c.waypoint_yaw = genfromtxt('waypoint_file_race_yaw.csv', delimiter = '')
    print('AMCL Initialization')
    c.amclSubscriber()
    print('Displaying Path on Rviz')
    time.sleep(2)
    c.display_path()
    #print('IMU Bias Calibration')
    #c.run_imu_bias()
    print('GO!')
    time.sleep(4)
    itr = 0
    rate = rospy.Rate(150)
    while not rospy.is_shutdown():
        itr = itr + 1
        c.run()
        if itr % c.waypoint_rate  == 0:                  # 160
            c.waypoint_index = c.waypoint_index + 1
        c.run_tune(itr)
        if c.waypoint_index == c.waypoint_dx.size:
            print('FINISH!')
            c.run_stop()
            break
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
