#!/usr/bin/env python2
from std_msgs.msg import String
import rospy
from serial import Serial

ser = Serial("/dev/ttyUSB1", 9600)

rospy.init_node("serial_recv", disable_signals=True)
pub = rospy.Publisher("Now_station", String, queue_size=2)
rate = rospy.Rate(50)

while True:
    try:
        received_data = ser.readline()
        received_data.decode()
        received_data1 = received_data.replace("\n", "")
#        pub.publish(received_data1)
        pub.publish("test")
        rate.sleep()
        print(received_data1)
    except KeyboardInterrupt:    
        print ("Exception")
        break
