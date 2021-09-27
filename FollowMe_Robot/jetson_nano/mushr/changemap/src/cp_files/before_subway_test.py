#!/usr/bin/env python
from changemap.srv import *
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from serial import Serial
import time
import rospy
import os

STATION = {'YU':["EY_1F.launch", "318_test.launch"],\
           'IMDANG':["EY_318_5.launch", "318_test.launch"]} # map name list
GOAL_POS = {'YU':[(5.06419658661, -4.70525884628, 0, 0, 0, 0.468347504147, 0.088354434827), (3.191, -1.175, 0, 0, 0, 0, 0)],\
            'IMDANG':[(3.781, -0.237, 0, 0, 0, 0, 0), (3.191, -1.175, 0, 0, 0, 0, 0)]}  # map goal position
count = 0  # index for list(map name)
once_first_info = True    # for first map
once_goal = True # for pub goal info once
once_map = True # for pub map info once
start = "YU"    # start map list (in future, from raspi)
end = "IMDANG"  # end map list (in future, from raspi)
state = start    # select between start and end

def select_map(data):
    global pub_sub, count, pub_rhc, state, pub_cb, once_first_info, once_goal, once_map, ser_map, ser, pub_safety, ser_vesc, ser_tts
    
    before_mapName=STATION[state][count].split("_")
    turn_180=before_mapName[0]

    flag = data.data   # flag 0 = no arrive, flag 1 = arrive
    if flag == 0 and once_goal == True:
        once_goal = False
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = GOAL_POS[state][count][0]
        goal.pose.position.y = GOAL_POS[state][count][1]
        goal.pose.position.z = GOAL_POS[state][count][2]

        goal.pose.orientation.x = GOAL_POS[state][count][3]
        goal.pose.orientation.y = GOAL_POS[state][count][4]
        goal.pose.orientation.z = GOAL_POS[state][count][5]
        goal.pose.orientation.w = GOAL_POS[state][count][6]
        print("pub cb_goal")
        pub_cb.publish(goal)
        once_map = True

    elif flag == 1 and once_map == True:
        once_map = False
        count += 1
        print("count = ", count)
        # general execution
        if (count >= 0 and count < len(STATION[state]) and state == start) or (count >= 0 and count < (len (STATION[state]) - 1) and state == end):
            map_name = STATION[state][count]
            print ("rest")
            ser_tts.write("ONELEVATOR tts\r\n".encode())
            ser.write("YU/floor3 ELV\r\n".encode())
            ser_tts.write("INELEVATOR tts\r\n".encode())
            print ("serial write")
            time.sleep (30)
#######################[ EDIT for turn_180 ]#########################
            if turn_180 == "EY":
                data="90 vesc\r\n"
                send_data=data.encode()
                ser_vesc.write(send_data) 
                time.sleep(2)
                turn_message = AckermannDriveStamped()
                turn_message.drive.speed=1
                turn_message.header.frame_id="turn"
                print ("start turn_180")
                ser_tts.write("TURN180 tts\r\n".encode())
                for i in range(35):
                    pub_safety.publish(turn_message)
                    time.sleep(0.1)
                print ("end turn_180")
                time.sleep (60)
                print ("end rest")
#####################################################################
            print("map_name = ", map_name)
            response=ser_map(map_name, False)
            print(response)

        # exception: out of range in list
        elif count >= len(STATION[state]) and state != end:
            ## for riding subway ##
            send_data_finish = "BSUB jetson\r\n"
            send_data_finish = send_data_finish.encode ()
            ser.write (send_data_finish)
            ## for riding subway ##
            count = 0
            state = end
            map_name = STATION[state][count]
            print ("subway map", count)
            print ("next map name", map_name)
#######################[ EDIT for turn_180 ]#########################
            if turn_180 == "EY":
                data="90 vesc\r\n"
                send_data=data.encode()
                ser_vesc.write(send_data) 
                time.sleep(2)
                turn_message = AckermannDriveStamped()
                turn_message.drive.speed=1
                turn_message.header.frame_id="turn"
                for i in range(50):
                    pub_safety.publish(turn_message)
                    time.sleep(0.1)
                time.sleep (60)
#####################################################################
            response=ser_map(map_name, False)
            print(response)
            pub_rhc.publish(count)

        elif count >= (len(STATION[state]) - 1) and state == end:
            print ("Really Goal Set!")
            send_data_finish = "Goal_set jetson\r\n"
            send_data_finish = send_data_finish.encode ()
            ser.write (send_data_finish)
            pub_rhc.publish(1111)
            os.system ("rosnode kill /c2g_map")
            os.system ("rosnode kill /map_server")
            os.system ("rosnode kill /particle_filter")
            os.system ("rosnode kill /rhcontroller")
            os.system ("rosrun changemap test.py")
        once_goal = True

def talker():
    global rate, pub_sub, pub_rhc, pub_cb, pub_test, pub_safety
    # publish map name to sub_changemap.py
    # publish "out of range in map list" to rhc_node.py
    pub_rhc = rospy.Publisher("Change_map", Int32, queue_size=2)
    # publish goal position to rhc_node.py(cb_goal)
    pub_cb = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 2)
    pub_safety = rospy.Publisher("/mux/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=2)

    # subscribe map count from rhc_node.py
    sub_rhc = rospy.Subscriber("count", Int32, select_map)
    rate.sleep()

if __name__ == "__main__":
    rospy.init_node("serial_recv", disable_signals=True, anonymous=True)
    ser_map=rospy.ServiceProxy('map_service', MapChangeInfo)
    ser = Serial("/dev/ttyUSB1", 9600)
    ser_vesc = Serial("/dev/ttyUSB2", 9600)
    ser_tts = Serial("/dev/ttyUSB3", 9600)
    rate = rospy.Rate(50)
    while True:
        try:
            received_data = ser.readline ()
            received_data.decode ()
            decoded_data = received_data.replace ("\r\n", "")
            print ("decode_data = ", decoded_data)
            if once_first_info==True:
               data, option = decoded_data.split(' ')
               start, end = data.split('/')
               print ("start, end = ", start, end)
               state = start
               print(STATION[state][count])
               response=ser_map(STATION[state][count], True)
               print("response = ", response)
               once_first_info=False
            talker()
            rate.sleep ()
        except KeyboardInterrupt:    
            print ("Exception")
            break
