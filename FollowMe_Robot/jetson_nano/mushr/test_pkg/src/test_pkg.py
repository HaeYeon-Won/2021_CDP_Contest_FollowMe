#!/usr/bin/env python
import rospy
import threading
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from std_msgs.msg import String
import hmac
import hashlib
import Jetson.GPIO as GPIO
import time
goal_position = {"home":[0,0,0,0,0,0,0], "304":[0,0,0,0,0,0,0]}
goal_name = ""
compare_L=[]
before_goal = ""

GPIO.setmode(GPIO.BCM)
mode = GPIO.getmode()
GPIO.setwarnings(False)
buzzer_pin = 4
GPIO.setup(buzzer_pin, GPIO.OUT)
servo_pin = 12
GPIO.setup(servo_pin, GPIO.OUT)
def servo_open():
    for _ in range(1500):
        GPIO.output(12, GPIO.HIGH)
        GPIO.output(12,GPIO.LOW)

def servo_close():
    for _ in range(100):
        GPIO.output(12, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(12,GPIO.LOW)

servo_open()
def callback(data):
    count = data.data
    print(count)
    if count == 1:
        goal = rospy.Subscriber("goal_topic",String, goal_dict, queue_size=1)

def QRsound():
    GPIO.output(buzzer_pin, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(buzzer_pin, GPIO.LOW)

def goal_dict(data):
    global compare_L, before_goal
    total = data.data
    if total not in compare_L and before_goal != total:
        compare_L.append(total)
        QRsound()
        print("compare !!!!!!!!!!!!!!!!!!!! ",compare_L)
        goal_name = compare(compare_L)
        print ("goal_name", goal_name)
        while True:
            if goal_name != "":
                print(goal_name)
                goal_value = goal_position[goal_name]
                print(goal_value)
                goal = PoseStamped()

                goal.header.seq = 1
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "map"

                goal.pose.position.x = goal_value[0]
                goal.pose.position.y = goal_value[1]
                goal.pose.position.z = goal_value[2]

                goal.pose.orientation.x = goal_value[3]
                goal.pose.orientation.y = goal_value[4]
                goal.pose.orientation.z = goal_value[5]
                goal.pose.orientation.w = goal_value[6]

                rospy.sleep(1)
                goal_publisher.publish(goal)
                break
        goal_name=""

def compare(tt):
    global compare_L, before_goal
    if len(tt) >= 2:
        first_header = tt[-1][:3]
        second_header = tt[-2][:3]
        print("fh!!!!!!!!!!!!!!!!!!!!!!!!",first_header,second_header)
        if first_header == second_header:
            print("sha check first: ",tt[-1][4:])
            print("sha check first_sha apply: ",hmac_sha256(tt[-1][4:]))
            print("sha check second: ", tt[-2][4:])
            if hmac_sha256(tt[-1][4:]) == tt[-2][4:]:
                before_goal = tt[-1]
                del compare_L[:]
                servo_open()
                g = "home"
                print("g!!!!!!!!!!!!!",g)
                return g
    if len(tt) == 1:
        servo_close()
        return tt[0][:3]

def hmac_sha256(data):
    API_SECRET = b"nsHc6458"
    signature=hmac.new(API_SECRET, data, digestmod=hashlib.sha256).hexdigest()
    return signature

#goal = rospy.Subscriber("goal_topic",String, goal_dict)
sub = rospy.Subscriber("ACK_atgoal",Int32,callback)
goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)

rospy.init_node("test_pkg")

goal = PoseStamped()

goal.header.seq = 1
goal.header.stamp = rospy.Time.now()
goal.header.frame_id = "map"

goal.pose.position.x = 3.191
goal.pose.position.y = -1.175
goal.pose.position.z = 0

goal.pose.orientation.x = 0
goal.pose.orientation.y = 0
goal.pose.orientation.z = 0
goal.pose.orientation.w = 0

goal_publisher.publish(goal)
print ("pub clear")

rospy.Rate(50)
rospy.sleep(1)
rospy.spin()
