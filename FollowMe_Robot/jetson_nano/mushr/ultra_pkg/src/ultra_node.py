#!/usr/bin/env python2
import Jetson.GPIO as GPIO
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import rospy
import threading
import serial

ser = serial.Serial("/dev/ttyUSB0", 115200)
cnt = 0
cnt_exception = 0

FORWARD = 1
STOP = 0
LIMIT_DISTANCE = 100
MAX_ROUND_TIME = 0.1

def getTFminiData():
    count = ser.in_waiting
    
    if count > 8:
        recv = ser.read(9)
        ser.reset_input_buffer()

        if recv[0] == 0x59 and recv[1] == 0x59:     #python3
            distance = recv[2] + recv[3] * 256
            strength = recv[4] + recv[5] * 256
            print('(', distance, ',', strength, ')')
            ser.reset_input_buffer()
            return distance

        if recv[0] == 'Y' and recv[1] == 'Y':     #python2
            lowD = int(recv[2].encode('hex'), 16)
            highD = int(recv[3].encode('hex'), 16)
            lowS = int(recv[4].encode('hex'), 16)
            highS = int(recv[5].encode('hex'), 16)
            distance = lowD + highD * 256
            strength = lowS + highS * 256
            print(distance, strength)
            return distance

def echo_action (channel):
    global start, end
    if (GPIO.input (ECHO) == GPIO.HIGH):
        start = time.time()
    else:
        end = time.time()


def callback(data):
    global imu_data
    imu_data=data.data
    #rospy.loginfo("execut callback")
    #rospy.loginfo(imu_data)

def talker():
    global count, pub, sub, imu_data, dist, rate
    rospy.init_node('ultra_node')
    pub = rospy.Publisher('safety_topic',Int32, queue_size =2) 
    sub = rospy.Subscriber('imu_topic',Int32, callback)
    rate = rospy.Rate(50) #10hz  
    dist = 0
    threading.Thread(target=thread).start()
    rospy.spin()

def thread():
    global count, pub, sub, imu_data, dist, rate, cnt, cnt_exception
    
    while not rospy.is_shutdown():
        if imu_data == STOP:
            rospy.loginfo("due to imu - stop")
            pub.publish(STOP)
        else:
            dist = getTFminiData()
            #rospy.loginfo(dist)
            if 0 < dist < LIMIT_DISTANCE:
                cnt+=1
                if cnt > 3:
                    rospy.loginfo("false!!!!!!!!!!!!")
                    pub.publish(STOP)
                    time.sleep(2)
            else:
                if cnt > 3:
                    cnt_exception+=1
                    if cnt_exception > 15:
                        cnt = 0
                        cnt_exception = 0
                        #rospy.loginfo(dist)
                        pub.publish(FORWARD)
                else:
                    cnt = 0
                    #rospy.loginfo(dist)
                    pub.publish(FORWARD)
            rate.sleep()

if __name__ == '__main__':
    try:
        if ser.is_open==False:
            ser.open()
        imu_data=FORWARD
        talker()
    except rospy.ROSInterruptException:
	if ser != None:
            ser.close()
        pass
