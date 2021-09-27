#!/usr/bin/env python

from changemap.srv import *
import rospy
import time

def handle_mapChange(data):
    for i in range(10):
        time.sleep(1)
        print(i)
    print("server start")
    print ("string_name", data.name)
    print ("first_info", data.first_info)
    return MapChangeInfoResponse(True)

def talker():
    ser = rospy.Service ('test_service', MapChangeInfo, handle_mapChange)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node ('test_service_server')
    talker()
