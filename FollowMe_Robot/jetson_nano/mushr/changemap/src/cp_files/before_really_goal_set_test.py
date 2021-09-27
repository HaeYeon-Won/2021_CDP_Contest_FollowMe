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

STATION = {'YU':["X-3F_YUsubway.launch", "X-1F_YU_X_EO1.launch", "TR180-1F_YU_EIU_TR.launch", "X-3F_YU_ET_SO.launch", "TR180-3F_YUsubway_SI_TR.launch"],\
           'IMDANG':["X-3F_IMsubway_ST_EO3.launch", "TR180-3F_IM_EID_TR.launch", "X-1F_IM_ET_END.launch", "S-Y_IMsubway.launch"]} # map name list
GOAL_POS = {'YU':[(),(2.33038306236,-0.195013567805, 0, 0, 0, 0.998855916065, 0.0478211139683), (-0.505123972893, 0.0791943073273, 0, 0, 0, 0.994840170674, 0.101454594832), (-3.85146713257, 2.80603051186, 0, 0, 0, 0.914588068305, -0.404386776878)],\
        'IMDANG':[(1.71381270885, -0.278054997325, 0, 0, 0, 0.998094507835, -0.061703755396), (-0.43142914772, -0.0866117477417, 0, 0, 0, 0.999999920942, -0.000397638468073), (7.011, -5.007, 0, 0, 0, -0.726, 0.688)]}  # map goal position
GOAL_SUBWAY = {'YUsubway':[(-4.88390254974, -0.0756535394287, 0, 0, 0, -0.738924657404, 0.673788060655),()], 'IMsubway':[(),()]}
MAP_SUBWAY=""
count = 0  # index for list(map name)
once_first_info = True    # for first map
once_goal = True # for pub goal info once
once_map = True # for pub map info once
#start = "YU"    # start map list (in future, from raspi)
#end = "IMDANG"  # end map list (in future, from raspi)
subway = 'SUBWAY'
#state = start    # select between start and end


def odom_turn(map_OT_info):
    global ser_vesc
    print("start OT_info, map_OT_info = ", map_OT_info)
    for i in map_OT_info:
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
            turn_message.drive.speed=-0.175
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
                turn_message.drive.speed=0.175
                for _ in range(18):
                    pub_safety.publish(turn_message)
                    time.sleep(0.1)
        print(" end ", i)
    print("end all OT_info")

def speak_tts(data):
    global ser_tts

    print ("data = ", data)
    if data == 'EIU':
        ser.write("YU/elv ELV\r\n".encode())
        ser_tts.write ("INELEVATOR tts\r\n".encode ())
        print ("INELEVATOR write")
    elif data == 'EID':
        ser.write("IM/elv ELV\r\n".encode())
        ser_tts.write ("INELEVATOR tts\r\n".encode ())
        print ("INELEVATOR write")
    elif data == 'EO1':     
        ser.write("YU/floor1 ELV\r\n".encode())
        ser_tts.write ("ONELEVATOR tts\r\n".encode ())
        print ("ONELEVATOR write")
    elif data == 'EO3':     
        ser.write("IM/floor3 ELV\r\n".encode())
        ser_tts.write ("ONELEVATOR tts\r\n".encode ())
    elif data == 'ET':
        ser_tts.write ("OUTELEVATOR tts\r\n".encode ())
        print ("OUTELEVATOR write")
    elif data == 'SO':
        ser_tts.write ("ONSCREEN tts\r\n".encode ())
        print ("ONSCREEN write")
    elif data == 'SI':
        ser_tts.write ("INSCREEN tts\r\n".encode ())
        print ("INSCREEN write")
    elif data == 'ST':
        ser_tts.write ("OUTSCREEN tts\r\n".encode ())
        print ("OUTSCREEN write")
    elif data == 'TR':
        ser_tts.write ("TURN180 tts\r\n".encode ())
        print ("TURN180 write")
    elif data == 'END':
        ser_tts.write ("END tts\r\n".encode ())
        print ("END write")


def select_map(data):
    global pub_sub, count, pub_rhc, state, pub_cb, once_first_info, once_goal, once_map, ser_map, ser, pub_safety, ser_vesc, SUBWAY_IN, start, end

    print ("map change start")
    print(" NOW state, count : ", state, count)
    if SUBWAY_IN != subway:
        before_mapName=STATION[state][count].split("_")
    else:
        before_mapName=MAP_SUBWAY.split("_")
    map_OT_info=before_mapName[0]
    OT_info = map_OT_info.split("-") # list
    subway_info=before_mapName[1]
    elevator_info_start = before_mapName[2]
    elevator_info_end = before_mapName[3]
    elevator_end_data, elevator_launch_data = elevator_info_end.split('.')
    print ("ele start = ", elevator_info_start)
    print ("ele end = ", elevator_end_data)
    
    flag = data.mapgoal   # flag 0 = no arrive, flag 1 = arrive
    if flag == 0 and once_goal == True:
        print("goal part in")
        once_goal = False
        if SUBWAY_IN != subway:
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
        else:
            goal = PoseStamped()
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = GOAL_SUBWAY[subway_info][count][0]
            goal.pose.position.y = GOAL_SUBWAY[subway_info][count][1]
            goal.pose.position.z = GOAL_SUBWAY[subway_info][count][2]

            goal.pose.orientation.x = GOAL_SUBWAY[subway_info][count][3]
            goal.pose.orientation.y = GOAL_SUBWAY[subway_info][count][4]
            goal.pose.orientation.z = GOAL_SUBWAY[subway_info][count][5]
            goal.pose.orientation.w = GOAL_SUBWAY[subway_info][count][6]
            pub_cb.publish(goal)
            print("pub cb_goal")
        goal_set = rospy.ServiceProxy('goal_set', GoalSet)
        res = goal_set("wait_goal")
        print(res)
        speak_tts(elevator_info_start)
        once_map = True
        
    elif flag == 1 and once_map == True:
        print("goal arrive part in")
        once_map = False
        count += 1
        print("count = ", count)
        # general execution
        if (count >= 0 and count < len(STATION[state]) and state == start and SUBWAY_IN != subway) or (count >= 0 and count < (len (STATION[state]) - 1) and state == end and SUBWAY_IN != subway):
            map_name = STATION[state][count]
            print ("start rest")
            ############################[EDIT for OT_info (by su-young)]########################### 
            speak_tts(elevator_end_data)
            print ("end speak_tts")
            OT_info.pop()
            odom_turn(OT_info)
            print("map_name = ", map_name)
            response=ser_map(map_name, False)
            print(response)

        # exception: subway
        elif count >= len(STATION[state]) and state == start and SUBWAY_IN != subway:
            ## for riding subway ##
            speak_tts(elevator_end_data)
            print ("end speak_tts")
            OT_info.pop()
            odom_turn(OT_info)
            print("#####[BSUB]#####")
            send_data_finish = "BSUB subway\r\n"
            send_data_finish = send_data_finish.encode ()
            ser.write (send_data_finish)
            print("write ", send_data_finish)
            canigo = ser.readline ()
            canigo.decode ()
            canigo_decode = canigo.replace ("\r\n", "")
            print ("canigo??", canigo_decode)
            c_data, c_option = canigo.split (' ')
            move, seat = c_data.split ('/')
            if move == 'GO':
                if seat == '1':
                    count = 0
                elif seat == 'FAIL':
                    count = 1
                print("subway count : ", count)
                map_name = MAP_SUBWAY
                response = ser_map(map_name, False)
                print (response)
                print ("subway map", map_name)
                SUBWAY_IN = subway

        elif SUBWAY_IN == subway:
            # change next station
            print("#####[ASUB]#####")
            speak_tts(elevator_end_data)
            subway_out = ser.readline()
            subway_out.decode()
            subway_decode = subway_out.replace("\r\n", "")
            print("OUT/ASUB : ", subway_decode)
            print("type : ", type(subway_decode))
            s_data, s_option = subway_decode.split (' ')
            print ("subway_data and option = ", s_data, s_option)
            if s_data == 'OUT/AUSB':
                count = 0
                state = end
                SUBWAY_IN="outside"
                map_name = STATION[state][count]
                print ("index ~~", count)
                print ("next map name", map_name)
                response=ser_map(map_name, False)
                print(response)
                pub_rhc.publish(count)

        elif count >= (len(STATION[state]) - 1) and state == end and SUBWAY_IN != subway:
            speak_tts(elevator_end_data)
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
    global SUBWAY_IN, start, end
    SUBWAY_IN=" "
    pub_usb_vesc = rospy.Publisher ("Pub_USB_Vesc", Int32, queue_size = 2)
    pub_usb_tf = rospy.Publisher ("Pub_USB_TF", Int32, queue_size = 2)
    rospy.init_node("serial_recv", disable_signals=True, anonymous=True)
    ser_map=rospy.ServiceProxy('map_service', MapChangeInfo)
    srv_map=rospy.Service('srv_map', MapandGoal, select_map)
    ser1 = Serial ("/dev/ttyUSB1", 9600)
    ser2 = Serial ("/dev/ttyUSB2", 9600)
    ser3 = Serial ("/dev/ttyUSB3", 9600)
    ser4 = Serial ("/dev/ttyUSB4", 9600)
    print ("data type", type(ser1))
    ser1.write ("jetson init\r\n".encode ())
    print ("ser1 write")
    ser2.write ("jetson init\r\n".encode ())
    print ("ser2 write")
    ser3.write ("jetson init\r\n".encode ())
    print ("ser3 write")
    ser4.write ("jetson init\r\n".encode ())
    print ("ser4 write")
    ser1_data = ser1.read ()
    print ("ser1_decode = ", ser1_data)
    ser2_data = ser2.read ()
    print ("ser2_decode = ", ser2_data)
    ser3_data = ser3.read ()
    print ("ser3_decode = ", ser3_data)
    ser4_data = ser4.read ()
    print ("ser4_decode = ", ser4_data)

    if ser1_data == "c":
        ser = ser1
        print ("1 is ser")
    elif ser1_data == "v":
        ser_vesc = ser1
        pub_usb_vesc.publish (1)
        print ("1 is vesc")
    elif ser1_data == "t":
        ser_tts = ser1
        print ("1 is tts")
    else:
        ser_tf = ser1
        print ("1 is tf")

    if ser2_data == "c":
        ser = ser2
        print ("2 is ser")
    elif ser2_data == "v":
        ser_vesc = ser2
        print ("2 is vesc")
    elif ser2_data == "t":
        ser_tts = ser2
        print ("2 is tts")
    else:
        ser_tf = ser2
        pub_usb_tf.publish (2)
        print ("2 is tf")

    if ser3_data == "c":
        ser = ser3
        print ("3 is ser")
    elif ser3_data == "v":
        ser_vesc = ser3
        print ("3 is vesc")
    elif ser3_data == "t":
        ser_tts = ser3
        print ("3 is tts")
    else:
        ser_tf = ser3
        print ("3 is tf")

    if ser4_data == "c":
        ser = ser4
        print ("4 is ser")
    elif ser4_data == "v":
        ser_vesc = ser4
        pub_usb_vesc.publish (4)
        print ("4 is vesc")
    elif ser4_data == "t":
        ser_tts = ser4
        print ("4 is tts")
    else:
        ser_tf = ser4
        print ("4 is tf")
    #ser.write("ser\r\n".encode())
    #ser_vesc.write ("ser_vesc\r\n".encode())
    #ser_tts.write ("ser_tts\r\n".encode())

    #ser = Serial("/dev/ttyUSB2", 9600)
    #ser_vesc = Serial("/dev/ttyUSB3", 9600)
    #ser_tts = Serial("/dev/ttyUSB1", 9600)
    rate = rospy.Rate(50)
    while True:
        try:
            if once_first_info==True:
                received_data = ser.readline ()
                received_data.decode ()
                decoded_data = received_data.replace ("\r\n", "")
                print ("decode_data = ", decoded_data)
                print("once_first_info : ", once_first_info)
                data, option = decoded_data.split(' ')
                start, end = data.split('/')
                print ("start, end = ", start, end)
                state = start
                MAP_SUBWAY = STATION[start].pop()
                STATION[start].pop(0)
                GOAL_POS[start].pop(0)
                STATION[end].pop()
                print ("station = ", STATION)
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
