import socket 
import threading 
import time
import serial
import spidev
import os
import RPi.GPIO as GPIO
import speech_recognition as sr
import playsound
import pygame
from gtts import gTTS
pygame.init()
pygame.mixer.init()

MACHINE_NUM=1
cs_lock=threading.Lock()
recv_data=''
test_tts=False
station_start, station_end='', ''

def convert_data (data):
    table = {'영남대':'YU', '임당':'IMDANG'}
    start, end =data.split('/')
    start = table.get (start)
    end = table.get (end)
    plan = start+'/'+end+' app'+'\r\n'
    #return 'YU/IMDANG\r\n' #for test
    return plan

def Send(client_sock):
    global flag, change, ser, MACHIN_NUM
    print("Start check Tx_Thread...")
    send_data = (str(MACHINE_NUM)+" "+"init").encode() #초기연결시 자신의 고유번호를 전송
    client_sock.send(send_data)
    while True:
        try:
            cs_lock.acquire()
            if change==1: #인터넷연결이 종료되었거나, IP주소가 변동되었으면 해당 Thread 종료
                cs_lock.release()
                break
            cs_lock.release()
            #jetson으로부터 시리얼을 통해 받아온 정보를 서버로 전송
            from_jetson=ser.readline()
            print(from_jetson)
            from_jetson=from_jetson.decode()
            from_jetson=from_jetson.replace("\r\n", "")
            data, option=from_jetson.split()
            if option=='jetson':
                send_data=data+'/'+str(MACHINE_NUM)+' jetson'
                client_sock.send(send_data.encode())
            elif option=='ELV':
                send_data=data+' ELV'
                client_sock.send(send_data.encode())
            
            elif option=='init':
                print(data, option)
                ser.write("c".encode())
                print("serial initialize complete")
            elif option=='subway':
                print(data, option)
                #ser.write("GO/1 server\r\n".encode())
                #print("write GO/1 server to jetson")
                client_sock.send(("1/"+data+" "+option).encode())
                #ser.write("OUT/AUSB server\r\n".encode())
                #print("write OUT/ASUB server to jetson")
        except UnicodeDecodeError:
            print("unicode Error")
            continue
        except ValueError:
            continue
            
            
def Recv(client_sock):
    global flag, change, ser, test_tts, station_start, station_end, MACHIN_NUM
    print("Start check Rx_Thread...")
    while True:
        try:
            print("Recv")
            cs_lock.acquire()
            if flag==-1: #인터넷연결이 종료되었거나, IP주소가 변동되었으면 해당 Thread 종료
                cs_lock.release()
                break
            cs_lock.release()
            #서버로 부터 온 정보를 jetson으로 전송
            recv_data = client_sock.recv(1024).decode()
            data, option=recv_data.split()
            if option=='app':
                print ("option is app")
                test_tts=True
                station_start, station_end = data.split('/')
                #save mp3
                txt="현재 이용하시는 로봇의 "+"출발역은 "+station_start+" 이고"+" 도착역은 "+station_end+" 입니다." + "출발지와 목적지가 옳바르다면, 손잡이에 버튼을 눌러주세요."
                filename=station_start+'_'+station_end+'.mp3'
                tts=gTTS(text=txt, lang='ko')
                tts.save(filename)
                ##
                send_data = convert_data (data)
                print ("send_data = ", send_data)
                ser.write (send_data.encode ())
            elif option=='server':
                print ("option is server")
                send_data = data+' '+option+'\r\n'
                print(send_data)
                ser.write(send_data.encode())
            elif option=='ACK':
                send_data="1 ACK"
                print("ACK")
                client_sock.send(send_data.encode())
        except ConnectionResetError: #연결리셋에러시 재연결 까지 대기
            flag=0
            change=1
            break

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
                print("IP addr changed!")
                cs_lock.release()
                break
        cs_lock.release()
        #print("Checking IP Addr...")
        time.sleep(1)
    
def checkInternetSocket(host="165.229.185.195", port=8080): #해당 호스트는 항상 연결되어있어야함 ex. 잘 알려진 인터넷 사이트로 하면될듯
    global flag
    print("Start check Network...")
    while True:
        try:
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port)) #해당 host에 연결시도
            #print("Internet connect...") 
            cs_lock.acquire()
            if flag==-1: #연결된 순간 flag를 다시 0으로 변경하여 다시 스레드들이 실행되게 해줌
                flag=0
            cs_lock.release()
        except socket.error as ex: 
            #실패하면 인터넷이 없는것 이므로 모든 스레드 종료 후 재연결떄까지 대기
            #print("Internet disconnect...")
            cs_lock.acquire()
           # print("check Internet")
            flag=-1
            cs_lock.release()
        time.sleep(1)

###########sensor and tts
Vcc = 5.0
R1 = 1000
# Open SPI bus
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = 500000

def fsr420_Registor(voltage):
    R = (R1 * Vcc)/voltage - R1
    return R

def ReadChannel(channel):
#  adc = spi.xfer2([1,(8+channel)<<4,0])
    adc = spi.xfer([1,(8+channel)<<4,0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data

# Define sensor channels(SS01)
mcp3008_channel = 0

# Define delay between readings
delay = 0.1
f = open('fsr402-2.dat', 'w')
index = 0

def test_tts():
    global station_start, station_end, test_tts
    while True:
        #analog_leveldd = ReadChannel(mcp3008_channel)
        #Vout = analog_level * Vcc / 1024.0
        #if(Vout < 2.2):
            #Vout = 0.001
            #Rfsr = 5000000
            #analog_level = 100
        #else:
            #Rfsr = fsr420_Registor(Vout)
        #print("digital = ", Vout)
        #f.write(data)
        if test_tts==True:
            while True:
                analog_level=ReadChannel(mcp3008_channel)
                print(analog_level)
                if analog_level>200:
                    pygame.mixer.music.load(station_start+'_'+station_end+'.mp3')
                    pygame.mixer.music.play()
                    while pygame.mixer.music.get_busy()==True:
                        continue
                    test_tts=False
                    break
                else:
                    for i in range(3):
                        pygame.mixer.music.load('beep.mp3')
                        pygame.mixer.music.play()
                        while pygame.mixer.music.get_busy()==True:
                            continue
                    pygame.mixer.music.load('START.mp3')
                    pygame.mixer.music.play()
                    while pygame.mixer.music.get_busy()==True:
                        continue
        else:
            pass

        time.sleep(2)
            


if __name__ == '__main__': 
    flag=0 #0: 연결 가능한 기본상태, 1:연결완료된 상태, -1:ip변동 또는 인터넷 끊김 상태
    change=0
    ser = serial.Serial ("/dev/ttyUSB0", 9600)
    CheckNetwort=threading.Thread(target=checkInternetSocket, args=()).start()
    tts_test=threading.Thread(target=test_tts, args=()).start()
    while True:
        try:
            if flag==1: #모든 스레드가 정상적으로 연결 완료된 상태
                time.sleep(1)
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
            time.sleep(1)
        except socket.error as ex: # 서버와 연결할 수 없으면 대기
            print("Server down")
            #print("flag = ", flag)
            time.sleep(1)
    
    

    
