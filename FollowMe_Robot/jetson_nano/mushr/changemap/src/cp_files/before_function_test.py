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

STATION = {'YU':["O-TR180-EY-O-TL90_1F_YU.launch", "TL90-O_3F_YU.launch"],\
           'IMDANG':["TR90-O-EY-TR180-O_3F_IM.launch", "X_1F_IM.launch"]} # map name list
GOAL_POS = {'YU':[(5.06419658661, -4.70525884628, 0, 0, 0, 0.468347504147, 0.088354434827), (0.032435387373, -0.057453930378, 0, 0, 0, -0.709332888191, 0.704873643805)],\
            'IMDANG':[(3.781, -0.237, 0, 0, 0, 0, 0), (3.191, -1.175, 0, 0, 0, 0, 0)]}  # map goal position
GOAL_SUBWAY = {'1':[], 'FAIL':[], 'OUT':[]}
MAP_SUBWAY = "map_subway.launch"
count = 0  # index for list(map name)
once_first_info = True    # for first map
once_goal = True # for pub goal info once
once_map = True # for pub map info once
start = "YU"    # start map list (in future, from raspi)
end = "IMDANG"  # end map list (in future, from raspi)
subway = 'SUBWAY'
state = start    # select between start and end

def select_map(data):
    global pub_sub, count, pub_rhc, state, pub_cb, once_first_info, once_goal, once_map, ser_map, ser, pub_safety, ser_vesc

    print ("map change start")
    
    before_mapName=STATION[state][count].split("_")
    map_OT_info=before_mapName[0]
    floor_info=before_mapName[1]
    flag = data.mapgoal   # flag 0 = no arrive, flag 1 = arrive
    if flag == 0 and once_goal == True:
        print("goal part in")
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
        pub_cb.publish(goal)
        print("pub cb_goal")
        once_map = True

    elif flag == 1 and once_map == True:
        print("goal arrive part in")
        once_map = False
        count += 1
        print("count = ", count)
        # general execution
        if (count >= 0 and count < len(STATION[state]) and state == start) or (count >= 0 and count < (len (STATION[state]) - 1) and state == end):
            map_name = STATION[state][count]
            print ("start rest")
            ser_tts.write("ONELEVATOR tts\r\n".encode())
            print ("onelevator")
            ser.write("YU/floor3 ELV\r\n".encode())
            ser_tts.write("INELEVATOR tts\r\n".encode())
            print ("inelevator")
            print ("serial write")
            ###########################[EDIT for OT_info (by su-young)]########################### 
            if map_OT_info != 'X':
                OT_info = map_OT_info.split("-") # list
            print("start OT_info, map_OT_info = ", map_OT_info)
            for i in OT_info:
                print(" start ", i)
                if i == 'O':
                    #odometry
                    #odom_message = AckermannDriveStamped()
                    #odom_message.drive.speed=1
                    #for j in range (15): # for 15 sec, go straight
                    #    pub_safety.publish(odom_message)
                    #    print("     time is ", 15 - j)
                    #    time.sleep(1)
                    for i in range (25):
                        time.sleep (1)
                        print ("time is ", 25 - i)
                elif i == 'EY': # EY
                    for j in range (50): # for 50 sec, wait in Elevator
                        time.sleep (1)
                        print("     time is ", 50 - j)
                elif i[0] == 'T': # TR180, TR90, TL90
                    #turn
                    data="90 vesc\r\n"
                    send_data=data.encode()
                    ser_vesc.write(send_data) 
                    time.sleep(2)
                    turn_message = AckermannDriveStamped()
                    turn_message.drive.speed=-1
                    turn_message.header.frame_id="turn"
                    if i == "TR180":
                        print ("TR180")
                        for _ in range(35):
                            pub_safety.publish(turn_message)
                            time.sleep(0.1)
                        
                    elif i == "TR90":
                        print ("TR90")
                        for _ in range(18):
                            pub_safety.publish(turn_message)
                            time.sleep(0.1)

                    elif i == "TL90":
                        print ("TL90")
                        turn_message.drive.speed=1
                        for _ in range(18):
                            pub_safety.publish(turn_message)
                            time.sleep(0.1)
                print(" end ", i)
            print("end all OT_info")

            print("map_name = ", map_name)
            response=ser_map(map_name, False)
            print(response)

        # exception: subway
        elif count >= len(STATION[state]) and state == start:
            ## for riding subway ##
            print ("BSUB")
            send_data_finish = "BSUB jetson\r\n"
            send_data_finish = send_data_finish.encode ()
            ser.write (send_data_finish)
            canigo = ser.readline ()
            canigo.decode ()
            canigo_decode = canigo.replace ("\r\n", "")
            print ("canigo??", canigo_decode)
            c_data, c_option = canigo.split (' ')
            move, seat = c_data.split ('/')
            if move == 'GO':
                print ("wait  gogo subway")
                time.sleep (20) # gogo subway
                # odometry part
                map_name = MAP_SUBWAY
                response=ser_map(map_name, False)
                print(response)

                seat_goal = PoseStamped ()
                seat_goal.header.seq = 1
                seat_goal.header.stamp = rospy.Time.now ()
                seat_goal.header.frame_id = "map"

                seat_goal.pose.position.x = GOAL_SUBWAY[seat][0]
                seat_goal.pose.position.y = GOAL_SUBWAY[seat][1]
                seat_goal.pose.position.z = GOAL_SUBWAY[seat][2]

                seat_goal.pose.orientation.x = GOAL_SUBWAY[seat][3]
                seat_goal.pose.orientation.y = GOAL_SUBWAY[seat][4]
                seat_goal.pose.orientation.z = GOAL_SUBWAY[seat][5]
                seat_goal.pose.orientation.w = GOAL_SUBWAY[seat][6]
                pub_cb.publish (seat_goal)
                print ("pub subway cb_goal")

                ## turn_180 ##
                data="90 vesc\r\n"
                send_data=data.encode()
                ser_vesc.write(send_data) 
                time.sleep(2)
                turn_message = AckermannDriveStamped()
                turn_message.drive.speed=-1
                turn_message.header.frame_id="turn"
                for i in range(35):
                    pub_safety.publish(turn_message)
                    time.sleep(0.1)
            ## for riding subway ##
            elif move == 'OUT':
                state = subway
                print ("goal subway station")
                map_name = MAP_SUBWAY
                response = ser_map(map_name, False)
                print (response)

                seat_goal = PoseStamped ()
                seat_goal.header.seq = 1
                seat_goal.header.stamp = rospy.Time.now ()
                seat_goal.header.frame_id = "map"

                seat_goal.pose.position.x = GOAL_SUBWAY[seat][0]
                seat_goal.pose.position.y = GOAL_SUBWAY[seat][1]
                seat_goal.pose.position.z = GOAL_SUBWAY[seat][2]

                seat_goal.pose.orientation.x = GOAL_SUBWAY[seat][3]
                seat_goal.pose.orientation.y = GOAL_SUBWAY[seat][4]
                seat_goal.pose.orientation.z = GOAL_SUBWAY[seat][5]
                seat_goal.pose.orientation.w = GOAL_SUBWAY[seat][6]
                pub_cb.publish (seat_goal)
                print ("pub subway cb_goal")

                # odometry part
                # go out subway
                data = "90 vesc\r\n"
                send_data = data.encode ()
                ser_vesc.write (send_data)
                time.sleep(2)
                turn_message = AckermannDriveStamped ()
                turn_message.drive.speed = -1
                turn_message.header.frame_id = "turn"
                for i in range (18):
                    pub_safety.publish (turn_message)
                    time.sleep (0.1)
                time.sleep (60)
                
        elif state == subway:
            # change next station
            count = 0
            state = end
            map_name = STATION[state][count]
            print ("index ~~", count)
            print ("next map name", map_name)
            response=ser_map(map_name, False)
            print(response)
            pub_rhc.publish(count)

        elif count >= (len(STATION[state]) - 1) and state == end:
            print ("Really Goal Set!")
            send_data_finish = "Goal_set jetson\r\n"
            send_data_finish = send_data_finish.encode ()
            ser.write (send_data_finish)
            # turn 180
            data = "90 vesc\r\n"
            send_data = data.encode ()
            ser_vesc.write (send_data)
            time.sleep(2)
            turn_message = AckermannDriveStamped ()
            turn_message.drive.speed = -1
            turn_message.header.frame_id = "turn"
            for i in range (18):
                pub_safety.publish (turn_message)
                time.sleep (0.1)
            # really goal set
            pub_rhc.publish(1111)
            os.system ("rosnode kill /c2g_map")
            os.system ("rosnode kill /map_server")
            os.system ("rosnode kill /particle_filter")
            os.system ("rosnode kill /rhcontroller")
            os.system ("rosrun changemap test.py")
        once_goal = True
        print ("map change complete")
    return MapandGoalResponse(1)


def talker():
    global rate, pub_sub, pub_rhc, pub_cb, pub_test, pub_safety, srv_count
    # publish map name to sub_changemap.py
    # publish "out of range in map list" to rhc_node.py
    pub_rhc = rospy.Publisher("Change_map", Int32, queue_size=2)
    # publish goal position to rhc_node.py(cb_goal)
    pub_cb = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 2)
    pub_safety = rospy.Publisher("/mux/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=2)
    rate.sleep()

if __name__ == "__main__":
    rospy.init_node("serial_recv", disable_signals=True, anonymous=True)
    ser_map=rospy.ServiceProxy('map_service', MapChangeInfo)
    srv_map=rospy.Service('srv_map', MapandGoal, select_map)
    ser = Serial("/dev/ttyUSB3", 9600)
    ser_vesc = Serial("/dev/ttyUSB1", 9600)
    ser_tts = Serial("/dev/ttyUSB2", 9600)
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
               srv_count = 0
            talker()
            rate.sleep ()
        except KeyboardInterrupt:    
            print ("Exception")
            break
