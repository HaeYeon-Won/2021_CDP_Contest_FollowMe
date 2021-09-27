#!/usr/bin/env python2
from serial import Serial
from std_msgs.msg import String
from time import sleep
import rospy
ser=Serial('/dev/ttyUSB1', 57600)
rospy.init_node ("tteesstt", disable_signals = True, anonymous = True)
rate = rospy.Rate (50)
while True:
    data='test gg'
    data+='\r\n'
    print ("data = ", data)
    s_data=data.encode()
    ser.write(s_data)
    sleep(1)
