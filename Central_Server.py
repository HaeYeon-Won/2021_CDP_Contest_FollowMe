import socket 
import threading 
from collections import deque
from time import sleep

def Send(grp, send_q): 
    global group, send_queue
    print('Thread Send Start') 
    while True: 
        try: 
            cs_queue.acquire()
            if len(send_queue)!=0:
                recv = send_queue.popleft() #받아온 Message중 가장 오래된 것 부터 전송
                flag=1 #전송할 Message가 있음을 알려주는 flag
            cs_queue.release()
             #새롭게 추가된 클라이언트가 있을 경우 Send 쓰레드를 새롭게 만들기 위해 루프를 빠져나감 
            if recv == 'Group Changed': 
                break 
        
            if flag==1: #만약 전송해야하는 Message가 있다면 전송
                cs_lock.acquire()
                for conn in group: 
                    msg =str(recv[0]) 
                    if recv[1] != conn: 
                        #client 자기자신을 제외한 클라이언트에게 Send
                        print("Send : ", msg)
                        conn.send(bytes(msg.encode()))
                flag=0 #전송이 완료되면 다시 flag는 0으로 변경하여 동일메시지의 추가전송을 막음
                cs_lock.release()
        except:
            pass
        sleep(1)

def Recv(conn, cnt, send_queue):
    global count, group #인원수와 그룹 구성원의 정보
    print('Thread Recv' + str(cnt) + ' Start') 
    while True:
        try:
            data = conn.recv(1024).decode()
            send_queue.append([data, conn, cnt]) #각각의 클라이언트의 메시지, 소켓정보, 구성원의 번호를 큐에 담아줌
        except ConnectionResetError: #
            #접속이 종료된 경우 구성원의 정보를 변경해줌
            print('Thread Recv' + str(cnt) + ' Close')
            cs_lock.acquire()
            print("remove : ", conn)
            group.remove(conn) #연결이 종료된 구성원의 정보를 리스트에서 삭제
            count-=1 #구성원의 수를 1 감소
            cs_lock.release()
            break
        
        sleep(1)


    
if __name__ == '__main__': 
    send_queue = deque()
    HOST = '192.168.0.2' #공유기 내부 아이피
    PORT = 8080 #포트포워딩한 포트
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP Socket 
    server_sock.bind((HOST, PORT)) # 소켓에 수신받을 IP주소와 PORT를 설정 
    server_sock.listen(10) # 소켓 연결, 접속 가능한 수 10
    count = 0
    group = [] #연결된 클라이언트 소켓 정보관리를 위한 리스트
    cs_lock=threading.Lock()
    cs_queue=threading.Lock()
    while True: 
        conn, addr = server_sock.accept() # 해당 소켓을 열고 대기 
        group.append(conn) #연결된 클라이언트의 소켓정보 저장
        print('Connected ' + str(addr))
        #print(group)
        if count==0: #초기상태
            start_rx=threading.Thread(target=Recv, args=(conn, count, send_queue)).start()
            start_tx=threading.Thread(target=Send, args=(group, send_queue)).start()
        elif count>=1: #추가적인 클라이언트 연결
            cs_queue.acquire()
            """
            클라이언트 구성원 정보가 변경됨을 알려줌
            이를 통해 현재 동작중인 tx스레드를 종료하고
            새로 그룹 구성원의 정보를 갱신 한 후 새로운 스레드를 실행함
            """
            send_queue.append('Group Changed') 
            cs_queue.release()
            cs_lock.acquire()
            start_rx=threading.Thread(target=Recv, args=(conn, count, send_queue)).start()
            start_tx=threading.Thread(target=Send, args=(group, send_queue)).start()
            cs_lock.release()
        count = count + 1 #구성원 수 추가
        sleep(0.5)

