#!/usr/bin/env python

import sys
import rospy
from changemap.srv import *

def mapChangeClient ():
    print ("client start")
    n = "abc"
    i = True
    cli = rospy.ServiceProxy ('test_service', MapChangeInfo)
    response = cli(n, i)
    print ("response = ", response)

if __name__ == "__main__":
    rospy.init_node ('test_service_client')
    mapChangeClient ()
