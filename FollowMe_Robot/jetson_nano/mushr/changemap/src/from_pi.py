#!/usr/bin/env python
from std_msgs.msg import String
from threading import Thread, Lock, Event
import rospy
from serial import Serial
import string
import sys

def tx_thread(ser, ev):
    print("tx start")
    while not ev.is_set():
        data=raw_input("Send Message : ")
        data+="\n"
        send_data=data.encode()
        ser.write(send_data)
        rate.sleep()

def rx_thread(ser, ev):
    print("rx start")
    while not ev.is_set():
        received_data = ser.readline()
        received_data.decode()
        received_data1 = received_data.replace("\n", "")
        pub.publish(received_data1)
        pub.publish("test")
        print(received_data1)
        rate.sleep()

if __name__ == "__main__":
    print("ready")
    ser = Serial("/dev/ttyUSB0", 9600)
    rospy.init_node("serial_recv", disable_signals=True, anonymous=True)
    rate = rospy.Rate(50)
    try:    
        ev=Event() 
        tx=Thread(target=tx_thread, args=(ser, ev, ))
        rx=Thread(target=rx_thread, args=(ser, ev, ))
        tx.start(); rx.start()
        ev.wait()
    except KeyboardInterrupt:
        print("end")
        ev.set()
        sys.exit(1)
