import socket
import serial
from time import sleep
def convert_data(data):
    table={'영남대':'YU', '임당':'IMDANG'}
    start, end=data.split()
    start=table.get(start)
    end=table.get(end)
    plan=start+' '+end
    return plan
        


#defime arg of serial
ser=serial.Serial("/dev/ttyUSB0", 9600)
client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #TCP Socket 생성
Host = '211.173.24.2' #서버 주소
Port = 8080 #포트포워딩한 포트번호
client_sock.connect((Host, Port)) #서버로 연결시도 
while True:
    """
    어플에서 서버로 보낸 데이터를 받아온 후 다시 jetson에 전달
    """
    recv_data = client_sock.recv(1024).decode()
    print("Recv : ", recv_data)
    Data=convert_data(recv_data)
    print("conver Data : ", Data)
    ser.write(Data.encode())
    sleep(0.1)


