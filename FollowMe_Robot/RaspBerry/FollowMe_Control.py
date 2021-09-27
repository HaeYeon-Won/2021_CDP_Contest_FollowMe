import RPi.GPIO as GPIO
from time import sleep
import serial
import speech_recognition as sr
import os
import pygame
from datetime import datetime
pygame.init ()
pygame.mixer.init ()

def change_servo_angle(ang, pin):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.OUT)
    p=GPIO.PWM(pin, 50)
    p.start(0)
    angle=-(ang//18)+7.4
    p.ChangeDutyCycle(angle)
    sleep(0.5)
    GPIO.cleanup()
    return

def change_vesc_angle(ang, pin):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.OUT)
    p=GPIO.PWM(pin, 50)
    p.start(0)
    angle=-(ang//18)+7.5
    p.ChangeDutyCycle(angle)
    sleep(10)
    GPIO.cleanup()
    return

ser = serial.Serial("/dev/ttyUSB1", 9600)
print("init Servo. . .")
while True:
    try:
        now=datetime.now()
        print("=====================================================")
        input_data = ''
        input_data = ser.readline()
        print("input_data = ", input_data, "{}:{}:{}".format(now.hour, now.minute, now.second))
        input_data = input_data.decode()  # Servo angle from Jetson
        decode_data = input_data.replace('\r\n', '')
        data, option = decode_data.split()

        print ("input_data, option  = ", data, option)
        if option=='servo':
            if data != '' and data[0] == '-':
                data = data[1:]
                data = float(data) * (-1)
            elif data != '':
                data=float(data)
            change_servo_angle(data, 11) # change angle to input_angle
        elif option=='vesc':
            if data!='' and data[0]=='-':
                data=data[1:]
                data=float(data)*(-1)
                print("vesc -90")
            elif data!='':
                data=float(data)
                print("vesc 90")
            change_vesc_angle(90, 15)
        elif option=='init':
            ser.write("v".encode())




    except KeyboardInterrupt:
        GPIO.cleanup()
        break
    except UnicodeDecodeError:
        print("unicode error")
        continue
