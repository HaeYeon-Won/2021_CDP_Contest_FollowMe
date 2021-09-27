#!/usr/bin/env python2

import rospy
import smbus
from time import sleep
from std_msgs.msg import Int32
 
class MPU6050:
    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = smbus.SMBus(1)

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18
 
    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    SELF_TEST_X = 0x0D
    SELF_TEST_Y = 0x0E
    SELF_TEST_Z = 0x0F
    SELF_TEST_A = 0x10

    ACCEL_XOUT0 = 0x3B
    ACCEL_XOUT1 = 0x3C
    ACCEL_YOUT0 = 0x3D
    ACCEL_YOUT1 = 0x3E
    ACCEL_ZOUT0 = 0x3F
    ACCEL_ZOUT1 = 0x40
 
    TEMP_OUT0 = 0x41
    TEMP_OUT1 = 0x42

    GYRO_XOUT0 = 0x43
    GYRO_XOUT1 = 0x44
    GYRO_YOUT0 = 0x45
    GYRO_YOUT1 = 0x46
    GYRO_ZOUT0 = 0x47
    GYRO_ZOUT1 = 0x48

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B


    def __init__(self, address):
        self.address = address
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    def read_i2c_word(self, register):
        high = self.bus.read_byte_data(self.address, register) # first register
        low = self.bus.read_byte_data(self.address, register + 1) # second reg

        value = (high << 8) + low 

        if (value >= 0x8000):

            return -((65535 - value) + 1)

        else:

            return value #

 

    # MPU-6050 Methods
    def get_temp(self): # 
        raw_temp = self.read_i2c_word(self.TEMP_OUT0) # 
        actual_temp = (raw_temp / 340) + 36.53 
        return actual_temp 

    def set_accel_range(self, accel_range): 
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw = False): # Accelerometer
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG) # 
        if raw is True: # 
            return raw_data
        elif raw is False: # Acce
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g = False): # Acc
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True) # return raw_data
        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            rospy.loginfo("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}

        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_range(self, gyro_range): # gyrosco
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range) # GYRO_

    def read_gyro_range(self, raw = False): # gyroscope
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG) #
        if raw is True:
            return raw_data
        elif raw is False: # Acceleromete
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self): # 
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True) # return raw_data
        if gyro_range == self.GYRO_RANGE_250DEG: #
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            rospy.loginfo("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier
        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        temp = get_temp()
        accel = get_accel_data()
        gyro = get_gyro_data()
        return [accel, gyro, temp]

if __name__=='__main__':
    try:
        sensor = MPU6050(0x68) # device addr
        pub=rospy.Publisher('imu_topic',Int32, queue_size=1)
        rospy.init_node('IMUsensor',anonymous=True)
        rate=rospy.Rate(10)

        while True:
            accel_data = sensor.get_accel_data()
            gyro_data = sensor.get_gyro_data()
            temp = sensor.get_temp()
 
            rospy.loginfo("temp data")
            rospy.loginfo("temp: "+str(temp))
        
            rospy.loginfo("Accelerometer data")
            rospy.loginfo("x: " + str(accel_data['x']))
            rospy.loginfo("y: " + str(accel_data['y']))
            rospy.loginfo("z: " + str(accel_data['z']))
        
            rospy.loginfo("Gyroscope data")
            rospy.loginfo("x: " + str(gyro_data['x']))
            rospy.loginfo("y: " + str(gyro_data['y']))
            rospy.loginfo("z: " + str(gyro_data['z']))
            rospy.loginfo("")

            THRESHOLD = 40

            if gyro_data['z'] > THRESHOLD or gyro_data['z'] < -THRESHOLD:
                rospy.loginfo("stop!")
                while(1):
                    speed_control=0
                    rospy.loginfo(speed_control)
                    pub.publish(speed_control)
                rate.sleep()
        

            sleep(0.5)

    except rospy.ROSInterruptException:
        pass
