#!/usr/bin/env python2
import pyzbar.pyzbar as pyzbar # pip install pyzbar 
import numpy as np # pip install numpy 
import cv2 # pip install opencv-python
import time
import Jetson.GPIO as GPIO
from std_msgs.msg import String
import rospy
import threading
cap = cv2.VideoCapture(0)
qr_cnt = 0
def decode(im): 
    global qr_data
    # Find barcodes and QR codes 
    decodedObjects = pyzbar.decode(im) 

    # Print results
    for obj in decodedObjects:
        print('Type : ', obj.type) 
        print('Data : ', obj.data, '\n')
        qr_data = obj.data
        print("decode : ", qr_data)
        time.sleep(0.8)
        return decodedObjects

# Display barcode and QR code location
def display(im, decodedObjects):
    print("display start")
    global qr_data, pub
    print(qr_data)
    # Loop over a ll decoded objects 
    for decodedObject in decodedObjects:
        points = decodedObject.polygon 

        # If the points do not form a quad, find convex hull
        if len(points) > 4:
            hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
            hull = list(map(tuple, np.squeeze(hull))) 
        else: hull = points; 

        # Number of points in the convex hull
        n = len(hull) 

        # Draw the convext hull 
        for j in range(0, n): 
            cv2.line(im, hull[j], hull[(j + 1) % n], (255, 0, 0), 3) 

    # Display results
    #cv2.imshow("Results", im)
    pub.publish(qr_data)
    print("Pub!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

def talker():
    global pub
    rospy.init_node('camera_node')
    pub = rospy.Publisher('goal_topic', String, queue_size=2)
    threading.Thread(target=camera_start).start()
    rate = rospy.Rate(50)
    rospy.spin()

def camera_start():
    while True:
        print("camera_pkg...start...")
        ret, frame = cap.read()
        #cv2.imshow('camera', frame)
        cv2.waitKey(1)
        decodedObjects = decode(frame)
        if decodedObjects is not None:
            display(frame,decodedObjects)
    cv2.destroyAllWindows()
# Main 
if __name__ == '__main__': 
    # Read image
    print("talker!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    talker()
