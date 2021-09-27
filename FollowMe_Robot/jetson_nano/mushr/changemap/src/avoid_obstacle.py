#!/usr/bin/env python2
import time
import serial
import math
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import *
import rospy
from threading import Thread, Lock, Event
import sys

BODY_LENGTH=55 #cm
SAFETY_DISTANCE=BODY_LENGTH*1.5 #cm
TIME_TO_WATI=3 #sec
OBSTACLE_APPEAR=0 #0:default(auto driving), 1:Dynamic obstacle  , -1:Static obstacle
MODE='waiting'
INIT_POS=True
move_angle=0
move_length = 0
NEW_STATICE = -1
state='go'
#can_go = 1

running_state = Event()
can_go = Event ()
cs_lock=Lock()

def send_data(data, option, ser):
    send_data=data+' '+option+'\r\n'
    print("Send_data = ", send_data)
    send_data=send_data.encode('utf-8')
    ser.write(send_data)
    return

def Get_Dist_From_Lidar():
    while True:
        try:
            count=ser.in_waiting
            print("count = ", count)
            if count>8:
                recv=ser.read(9)
                ser.reset_input_buffer()
                if recv[0] == 'Y' and recv[1] == 'Y':     #python2
                    lowD = int(recv[2].encode('hex'), 16)
                    highD = int(recv[3].encode('hex'), 16)
                    lowS = int(recv[4].encode('hex'), 16)
                    highS = int(recv[5].encode('hex'), 16)
                    distance = lowD + highD * 256
                    return distance
        except KeyboardInterrupt:
            exit()

def Avoid_obstacle(ser_usb):
    global OBSTACLE_APPEAR, move_angle, move_length, time_go, MODE, INIT_POS, NEW_STATIC, state, msg_drive, pub_stop, can_go, ser_tts
    static_obstacle=False
    count=0
    print ("Avoid_obstacle")
    while True:
        try:
            if state=='go' and MODE!='running':
                if INIT_POS==True:

                    Search_Table={} #TF_mini Lidar
                    move_range=[] #Search Tabl
                    temp_buff=[] #
                    print("wowwowowow")
                    dist=Get_Dist_From_Lidar()
                    print("efknlanfela")
                    time.sleep(0.1)
                    print("dist in Avoid_Obstacle = ", dist, "cm")

                    if dist<=SAFETY_DISTANCE and count>=3:
                        ser_tts.write ("STATICOBSTACLE tts\r\n".encode())
                        can_go.set ()
                        count=0
                        cs_lock.acquire()
                        NEW_STATIC=1
                        if MODE=='running':
                            running_state.set()
                        cs_lock.release()

                        for i in range(-70,71,20):
                            print("ang = ", i)
                            send_data(str(i), 'servo', ser_usb)
                            time.sleep(0.6)
                            #change_servo_angle(i)
                            Search_Table[i]=Get_Dist_From_Lidar()
                            print("TEST : ", i, Search_Table)
                        Search_Table=sorted(Search_Table.items())
                        print ("Search_Table = ", Search_Table)

                        for val in Search_Table:
                            if val[1]>=SAFETY_DISTANCE:
                                temp_buff.append(val[0])
                            else:
                                move_range.append(temp_buff)
                                temp_buff=[]
                        if len(temp_buff)!=0:
                            move_range.append(temp_buff)

                        move_range.sort(key=len) 
                        print ("move_range = ", move_range[-1])
                        try:
                            table_length = len(move_range[-1])
                            total_angle = sum(move_range[-1])
                            if table_length < 2:
                                table_length = 0
                            move_angle = total_angle / table_length 
                            move_length=int(SAFETY_DISTANCE//abs(math.cos(math.radians(move_angle)))) 
                            #move_length=30 #move_length for test
                            send_data(str(int(move_angle)), 'servo', ser_usb) 
                            INIT_POS=False 
                            print ("cccccccccccccc")
                            running_state.set() 

                        except ZeroDivisionError:
                            print ("Cannot go!!!!!!!!")
                            send_data('0', 'servo', ser_usb)
                            count = 3
                            continue
                    #static_obstacle=False 

                    elif dist<=SAFETY_DISTANCE: 
                        ser_tts.write ("DYNAMICOBSTACLE tts\r\n".encode())
                        #static_obstacle=True 
                        count+=1
                        #MODE='waiting'
                        running_state.clear() 
                        print("Time to Wait. . .")
                        #msg_drive.drive.speed = 0
                        #msg_drive.drive.steering_angle = 0
                        print ("drive stop!!")
                        #pub_stop.publish (msg_drive)
                        can_go.set ()
                        time.sleep(TIME_TO_WATI)
                    elif count>=1 and dist>SAFETY_DISTANCE:
                        count=0
                        can_go.clear ()
                    else:
                        can_go.clear ()
                        if MODE == 'running':
                            print ("Running State . . .")
                            running_state.set ()
                        elif MODE == 'finish' or MODE == 'waiting':
                            print ("Finish . . .")
                            running_state.clear ()
                else:
                    print("init robot Position . . .") 
                    time.sleep(0.5)

            elif state=='finish':
                dist=Get_Dist_From_Lidar()
                print("auto driving finish!!!!!!!  dist = ", dist)
        except KeyboardInterrupt:
            sys.exit(0)

def robot_stop():
    global can_go, pub_stop, msg_drive
    print("stop init")
    #can_go.wait ()
    print ("robot_stop")
    while True:
        if not can_go.isSet ():
            can_go.wait ()
        else:
            print ("robot stop moving!~!")
            msg_drive.drive.steering_angle = 0
            msg_drive.drive.speed = 0
            pub_stop.publish (msg_drive)
        time.sleep(0.1)

def move_robot(ser_usb):
    global move_angle, move_length, running_statem, MODE, INIT_POS, NEW_STATIC, msg_drive
    while True:
        try:
            print("Waiting to move . . .")
            running_state.wait()
            cs_lock.acquire()
            MODE='running' 
            NEW_STATIC=0
            cs_lock.release()
            print("move start. . .")
            #change_servo_angle(move_angle) 
            #send_data(str(move_angle), 'servo', ser_usb)
            time.sleep(3)
            print(INIT_POS)
            INIT_POS=True
            print(INIT_POS)
            print("move length = ", move_length)
            msg_drive.drive.speed=0.1
            msg_drive.drive.steering_angle=int(move_angle) * (-1)
            can_go.clear ()
            print("move length = ", move_length)
            send_data('0', 'serve', ser_usb)
            loop=move_length//2
            for i in range(loop+20): 
                if not running_state.isSet():
                    print("Obstacle Appear, wait a moment. . .")
                    running_state.wait()
                    if NEW_STATIC == 1:
                        print ("new static obstacle appear . . .")
                        break
                if i==5:
                    msg_drive.drive.speed=0.5
                print("Go to destination. . .")
                pub_stop.publish(msg_drive)
                print("iiiiiiiii = ",i)
                #if i == 2:
                #    send_data ('0', 'servo', ser_usb)
                #if i == 20:
                    #print ("move_angle is 00000")
                    #msg_drive.drive.steering_angle=0
                if i > 25:
                    print ("move_angle change")
                    msg_drive.drive.steering_angle = int(move_angle)
                time.sleep(0.1)

            cs_lock.acquire()
            MODE='finish'
            running_state.clear()
            cs_lock.release()
            print("move clear. . .")
        except KeyboardInterrupt:
            sys.exit(0)

def tf_state(data):
    global state
    if data.data == "finish":
        state = "finish" 
    elif data.data == "go":
        state = "go"

def Usb_Vesc (data):
    global ser_usb, vesc_init
    print ("Usb_Vesc", data.data)
    port = "/dev/ttyUSB" + str(data.data)
    ser_usb = serial.Serial (port, 9600)
    vesc_init=1

def Usb_Tf (data):
    global ser, tf_init
    print ("Usb_Tf", data.data)
    port = "/dev/ttyUSB" + str (data.data)
    ser = serial.Serial (port, 115200)
    tf_init=1

def Usb_tts (data):
    global ser_tts, tts_init
    print ("Usb_tts", data.data)
    port = "/dev/ttyUSB" + str (data.data)
    ser_tts = serial.Serial (port, 9600)
    tts_init=1
#Define TF mini Lidar args
ser = ""
#ser_usb=serial.Serial("/dev/SER_USB", 9600)
ser_usb = ""
ser_tts=''
vesc_init, tf_init,tts_init, init =0, 0, 0,0
#########################
rospy.init_node ("TF_stop", disable_signals = True, anonymous = True)
#send_data(str(0), 'servo', ser_usb)
msg_drive = AckermannDriveStamped()
pub_stop = rospy.Publisher ("mux/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size = 2)
sub_usb_tf = rospy.Subscriber ("Pub_USB_TF", Int32, Usb_Tf)
sub_usb_vesc = rospy.Subscriber ("Pub_USB_Vesc", Int32, Usb_Vesc)
sub_usb_tts = rospy.Subscriber ("Pub_USB_TTS", Int32, Usb_tts)
#s = Thread(target = robot_stop, args=()).start()
while True:
    try:
        print("__main__")
        if vesc_init==1 and tf_init==1 and tts_init==1 and init!=1: 
            send_data(str(0), 'servo', ser_usb)
            t = Thread(target = Avoid_obstacle, args=(ser_usb,)).start()
            g = Thread(target = move_robot, args=(ser_usb, )).start()
            s= Thread(target = robot_stop, args=()).start()
            init=1
#s = Thread(target = robot_stop, args=()).start()
        sub_go = rospy.Subscriber ("TF_state", String, tf_state)
        #sub_usb_vesc = rospy.Subscriber ("Pub_USB_Vesc", Int32, Usb_Vesc)
        time.sleep(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt!!!")
        sys.exit(0)
