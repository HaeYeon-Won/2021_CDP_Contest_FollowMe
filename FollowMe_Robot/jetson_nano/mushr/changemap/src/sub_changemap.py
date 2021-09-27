#!/usr/bin/env python
from changemap.srv import *
from std_msgs.msg import String
from changemap.msg import MapData
from std_msgs.msg import Int32
import rospy
import threading
import os

class Map(threading.Thread):
    def __init__(self, name):
        super(Map, self).__init__()
        self.name=name

    def run(self):
        print("thread get name : ", threading.currentThread().getName())
        if threading.currentThread().getName()=="True":
            os.system ("rosnode kill /c2g_map")
            os.system ("rosnode kill /map_server")
            print ("rosnode kill")
            os.system("roslaunch mushr_rhc_ros cp_real.launch")
        elif threading.currentThread().getName()=="False":
            print("pass")
        else:
            print (threading.currentThread().getName())
            os.system("roslaunch changemap " + threading.currentThread().getName())

def handle_mapChange(data):
    thread_map_launch=Map(data.name)
    thread_first_info=Map(data.first_info)
    print("map_name = ", data.name)
    print("first_info = ", data.first_info)
    thread_map_launch.start()
    thread_first_info.start()
    return MapChangeInfoResponse(True)

def receiver():
    ser_map = rospy.Service ('map_service', MapChangeInfo, handle_mapChange)
    rospy.spin()

if __name__ == "__main__":
    print("Sub ready!!")
    rospy.init_node("sub_changemap", disable_signals=True, anonymous=True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            receiver()
        except KeyboardInterrupt:
            print("Generete exception")
            break
