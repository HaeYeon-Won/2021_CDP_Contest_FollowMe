import socket 
import threading 
import time
#import serial
import RPi.GPIO as GPIO
import speech_recognition as sr
import os
import pygame
from gtts import gTTS
import playsound
pygame.mixer.init()

seat_flag = 0
Machine = "SEAT init"
GPIO.setmode(GPIO.BCM)
trig = 2
echo = 3
LED = 16
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)
GPIO.setup(LED, GPIO.OUT)

"""
while True:
    GPIO.output(17, False)
    time.sleep(2)
    GPIO.output(LED, True)
    time.sleep(2)
"""
cs_lock=threading.Lock()
recv_data=''

def ledctrl(pin):
    GPIO.output(pin, False)
    print ("led ON")
    time.sleep(1)

def ledctrl_down(pin):
    GPIO.output(pin, True)
    print ("led OFF")
    time.sleep(1)



def ultra_sonic(trig, echo):
        pulse_start = 0
        pulse_end = 0
        GPIO.output(trig, False)
        time.sleep(0.5)

        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)

        while GPIO.input(echo) == 0:
            pulse_start = time.time()

        while GPIO.input(echo) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17000
        distance = round(distance, 2)

        print("distance : ", distance, "cm")
        return distance
def convert_data (data):
    table = {'영남대':'YU', '임당':'IMDANG'}
    start, end =data.split('/')
    start = table.get (start)
    end = table.get (end)
    plan = start+'/'+end+' app'+'\r\n'
    return plan

def Send(client_sock):
    global flag, change
    send_data = Machine.encode()
    client_sock.send(send_data)
    print("Start check Tx_Thread...")
    while True:
        cs_lock.acquire()
        if change==1: #인터넷연결이 종료되었거나, IP주소가 변동되었으면 해당 Thread 종료
            cs_lock.release()
            print("break Send!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            break
        cs_lock.release()
        #jetson으로부터 시리얼을 통해 받아온 정보를 서버로 전송
        #from_jetson=ser.readline()
        #from_jetson=1
        #from_jetson=from_jetson.replace("\r\n", "")
        #jetson_data=from_jetson.decode()
        #data, option=jetson_data.split()
        #if option=='jetson':
        time.sleep(3)    
            
def Recv(client_sock):
    global flag, change, recv_data, seat_flag
    print("Start check Rx_Thread...")
    while True:
        try:
            cs_lock.acquire()
            if flag==-1: #인터넷연결이 종료되었거나, IP주소가 변동되었으면 해당 Thread 종료
                cs_lock.release()
                break
            cs_lock.release()
            #서버로 부터 온 정보를 jetson으로 전송
            recv_data = client_sock.recv(1024).decode()
            print(recv_data)
            data, option=recv_data.split()
            if option=='app':
                print("seat_flag = ", seat_flag)
                print("find seat")
                distance = ultra_sonic(2,3)
                if 30 < distance and distance <= 500:
                    ledctrl(16)
                    seat_flag = 1
                    send_data = '1 seat'
                    print(send_data)
                    client_sock.send(send_data.encode())
                elif distance > 1000 or distance <= 30 :
                    send_data='FAIL seat'
                    print(send_data)
                    client_sock.send(send_data.encode())

                #ser.write (send_data.encode ())
            elif option=='server':
                seat_flag = 0
                ledctrl_down(16)
                print ("option is server")
                send_data = "GO rasberry\r\n"
                #ser.write (send_data.encode ())
            elif option == 'ELV':
                print("TEST clear")
                setServoPos(20)
                time.sleep (1)
                setServoPos (0)
                time.sleep (1)
            elif option == 'ACK':
                client_sock.send("SEAT ACK".encode())

        except ConnectionResetError: #연결리셋에러시 재연결 까지 대기
            print("break in recv!!!!!!!!!!!!!!!!!!")
            flag=0
            change=1
            break

def speak(filename):
    pygame.mixer.music.load(filename)
    pygame.mixer.music.play()
    while(pygame.mixer.music.get_busy()):
        continue

def check_my_addr():
    global flag
    print("Start check IP_Addr...")
    IP_Addr=socket.gethostbyname(socket.getfqdn()) #현재 자신의 IP 주소 저장
    while True:
        cs_lock.acquire()
        if flag==-1: #인터넷 연결 x => break
            cs_lock.release()
            break
        cs_lock.release()

        cs_lock.acquire()
        if flag!=-1: #인터넷 연결은 되어있지만, 아이피 변동 => break
            if IP_Addr!=socket.gethostbyname(socket.getfqdn()):
                flag=-1
                cs_lock.release()
                break
        cs_lock.release()
        #print("Checking IP Addr...")
        time.sleep(0.5)
    
def checkInternetSocket(host="165.229.185.194", port=8080): #해당 호스트는 항상 연결되어있어야함 ex. 잘 알려진 인터넷 사이트로 하면될듯
    global flag
    print("Start check Network...")
    while True:
        try:
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port)) #해당 host에 연결시도
            #print("Internet connect...") 
            cs_lock.acquire()
            if flag==-1: #연결된 순간 flag를 다시 0으로 변경하여 다시 스레드들이 실행되게 해줌:
                flag=0
            cs_lock.release()
        except socket.error as ex: 
            #실패하면 인터넷이 없는것 이므로 모든 스레드 종료 후 재연결떄까지 대기
            #print("Internet disconnect...")
            cs_lock.acquire()
            flag=-1
            cs_lock.release()
        time.sleep(0.5)


if __name__ == '__main__':
    seat_flag=0
    ledctrl_down(16)
    flag=0 #0: 연결 가능한 기본상태, 1:연결완료된 상태, -1:ip변동 또는 인터넷 끊김 상태
    change=0
    CheckNetwort=threading.Thread(target=checkInternetSocket, args=()).start()
    #ser = serial.Serial ("/dev/ttyUSB0", 9600)
    while True:
        try:
            if seat_flag == 1:
                distance = ultra_sonic(2, 3)
                if distance > 1000 or distance <= 30 :
                        speak("SEAT.mp3")
                time.sleep(1)
            if flag==1: #모든 스레드가 정상적으로 연결 완료된 상태
                print("main sleep")
                time.sleep(0.5)
            elif flag==0: #인터넷 문제는 없고, 아직 스레드가 생성되지 않았을 때
                client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #TCP Socket 
                Host = '165.229.185.243' #통신할 대상의 IP 주소 
                Port = 8080 #통신할 대상의 Port 주소 
                client_sock.connect((Host, Port)) #서버로 연결시도 
                print("Client1")
                print('Connecting to ', Host, Port) #Client의 메시지를 보낼 쓰레드 
                tx_thread = threading.Thread(target=Send, args=(client_sock, )).start()
                rx_thread = threading.Thread(target=Recv, args=(client_sock, )).start()
                check_idAddr=threading.Thread(target=check_my_addr, args=()).start()
                flag=1 #실행완료후 main문은 대기
                change=0
        except socket.error as ex: # 서버와 연결할 수 없으면 대기
            print("cannot connect Server!!!!!!!!")
            print("flag = ", flag)
            time.sleep(1)
    
    

    
