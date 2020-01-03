#!/usr/bin/env python




import rospy
from std_msgs.msg import Float32
import time
# rally car hardware
import serial

console_serG = serial.Serial('/dev/ttyACM0',115200)


def main():
    ## Node and class initializations
    print('Iniitializing')
    rospy.init_node('Car_on_Q4', anonymous=True)
    time.sleep(7)
    cmd = 'A+0240+2000'
    console_serG.write(cmd)
    time.sleep(1.7)
    console_serG.write('A+2235+0000')
    print('Stopped')
    rospy.spin()

if __name__ == '__main__':
    main()
