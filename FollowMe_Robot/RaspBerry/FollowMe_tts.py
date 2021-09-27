import playsound
import pygame
import speech_recognition as sr
import os
import serial
from time import sleep
from datetime import datetime
pygame.init()
pygame.mixer.init()
ser=serial.Serial("/dev/ttyUSB2", 9600)
print("init tts OK")
while True:
    try:
        now=datetime.now()
        print("====================================================")
        input_data=''
        input_data=ser.readline()
        print("input data = ", input_data, "({}:{}:{}".format(now.hour, now.minute, now.second))
        input_data=input_data.decode()
        decode_data=input_data.replace("\r\n","")
        data, option=decode_data.split()
        print("input_data, option = ",data, option)

        if option=='tts':
            file_name=data+'.mp3'
            pygame.mixer.music.load(file_name)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy()==True:
                continue
            print("filename = ", file_name)
        elif option=='init':
            ser.write('t'.encode())
    except KeyboardInterrupt:
        break
    except UnicodeDecodeError:
        print("Unicode Error")
        continue
