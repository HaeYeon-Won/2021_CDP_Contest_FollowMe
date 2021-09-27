import speech_recognition as sr

from gtts import gTTS

import os

import time

import pygame

import playsound

pygame.init()

pygame.mixer.init()

 

 

def speak(txt, index):


    tts = gTTS(text=index, lang='ko')

    filename=txt+'.mp3'

    tts.save(filename)

    #pygame.mixer.music.load(filename)

    #pygame.mixer.music.play()

    #while pygame.mixer.music.get_busy()==True:

        #continue

    print("Create ", filename, "complete")

 

 

speak_dict = {"START":"손잡이를 잡아주세요.","END": "목적지에 도착했습니다. 안내를 종료하겠습니다.","ONELEVATOR": "엘리베이터 앞에 도착했습니다. 잠시 대기해 주세요.","ONSCREEN": "스크린 도어 앞에 도착했습니다. 잠시 대기해주세요.","DYNAMICOBSTACLE": "지나가겠습니다. 잠시 비켜주세요.","INSCREEN":"열차가 도착했습니다. 탑승 하겠습니다.", "INELEVATOR":"엘리베이터에 탑승하겠습니다.", "TURN180":"제자리 회전 하겠습니다. 회전방향에 맞춰 움직여 주세요.", "STATICOBSTACLE":"전방에 장애물이 발견되었습니다. 새로운 경로로 주행하겠습니다.", "OUTELEVATOR":"엘리베이터에서 내리겠습니다.", "RESERVATION":"예약하신 자리로 이동하겠습니다.","OUTBEFORESCREEN":"도착역 까지 한정거장 남았습니다.","OUTSCREEN":"잠시후 하차하겠습니다."}

 

print("init voice . wait a moment . . . ")

 

for key,val in speak_dict.items():

    speak(key, val)

print("Create finish!")
