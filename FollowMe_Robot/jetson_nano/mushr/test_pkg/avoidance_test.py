import time
import Jetson.GPIO as GPIO
import serial
from threading import Thread, Lock

#Define global args
SAFETY_DISTANCE=30 #cm
TIME_TO_WATI=1 #sec
FLAG=0
###################

def change_servo_angle(ang): #input -90~90
    angle=-(ang//18)+7.5
    p.ChangeDutyCycle(angle)
    time.sleep(0.5)

def Get_Dist_From_Lidar():
    print("Get dist")
    while True:
        count = ser.in_waiting
        if count > 8:
            recv = ser.read (9)
            ser.reset_input_buffer()
            if recv[0] == 0x59 and recv[1] == 0x59:
                distance = recv[2] + recv[3] * 256
                ser.reset_input_buffer()
                return distance

def Avoid_obstacle():
    print("clear")
    flag=0
    while(True):
        try:
            table={}
            move_range=[]
            temp=[]
            dist=Get_Dist_From_Lidar()
            print ("------------------------------------------------------------------------")
            print("dist = ", dist)
            print ("------------------------------------------------------------------------")
            if flag==1 and dist<=SAFETY_DISTANCE:
                flag=0
                for i in range(-90,91, 20): #0:-90, 179:90
                    print("angle changed")
                    change_servo_angle(i)
                    table[i]=Get_Dist_From_Lidar()
                print("table = ", table)
                for key, val in table.items():
                    if val>=SAFETY_DISTANCE:
                       temp.append(key)
                    else:
                        move_range.append(temp)
                        temp=[]
                if len(temp)!=0:
                    move_range.append(temp)
                print("Before sorting move range = " , move_range)
                move_range.sort(key=len)
                print("After sorting move_range : ", move_range)
                move_range=move_range[-1]
                print("Result move_range", move_range)
                if len(move_range)!=0:
                    move_angle=sum(move_range)/len(move_range)
                else:
                    print ("Can't go!")
                    print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                print("move Angle = ",move_angle)
                duty_cycle=-(move_angle//18)+7.5

            elif flag==0 and dist<=SAFETY_DISTANCE:
                flag=1
                print("TIME TO WAIT")
                time.sleep(TIME_TO_WATI)
            time.sleep(1)

        except KeyboardInterrupt:
            GPIO.cleanup()
            break




#Define Servo args
left_angle=12.5
right_angle=2.5
center_angle=7.5
Servo_pin=32
GPIO.setmode(GPIO.BOARD)
GPIO.setup(Servo_pin, GPIO.OUT)
p=GPIO.PWM(Servo_pin, 50)# freq 50
p.start(0)
change_servo_angle(0)
##################

#Define TF mini Lidar args
ser=serial.Serial("/dev/ttyUSB0", 115200)
#########################
Avoid_obstacle()

