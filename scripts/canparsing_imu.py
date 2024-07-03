#!/usr/bin/env python3

#-- Author: Anditya Sridamar Pratyasta : anditya.sridamar.p@kangwon.ac.kr


'''
TEST
canparsing_frame = "585"
if(data[7] == 0x00){
Roll  = (float)((int16_t)(data[1] << 8 | data[0])) / 100;
Pitch = (float)((int16_t)(data[3] << 8 | data[2])) / 100;
Yaw   = (float)((int16_t)(data[5] << 8 | data[4])) / 100;

else if(data[7] == 0x01){
AccX = (float)((int16_t)(data[1] << 8 | data[0])) / 1000.0f;
AccY = (float)((int16_t)(data[3] << 8 | data[2])) / 1000.0f;
AccZ = (float)((int16_t)(data[5] << 8 | data[4])) / 1000.0f;
};;;
else if(data[7] == 0x02){
GyroX = (float)((int16_t)(data[1] << 8 | data[0])) / 100.0f;
GyroY = (float)((int16_t)(data[3] << 8 | data[2])) / 100.0f;
GyroZ = (float)((int16_t)(data[5] << 8 | data[4])) / 100.0f;
}

'''

#git push testing

import rospy
from sensor_msgs.msg import Imu
import socket
import struct

def decode_can_frame(data):
    if data[7] == 0x00:
        roll = (float)((int16_t)(data[1] << 8 | data[0])) / 100
        pitch = (float)((int16_t)(data[3] << 8 | data[2])) / 100
        yaw = (float)((int16_t)(data[5] << 8 | data[4])) / 100
        rospy.loginfo("Decoded Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw)
        return roll, pitch, yaw
    elif data [7] == 0x01:
        acc_x = (float)((int16_t)(data[1] << 8 | data[0])) / 1000.0
        acc_y = (float)((int16_t)(data[3] << 8 | data[2])) / 1000.0
        acc_z = (float)((int16_t)(data[5] << 8 | data[4])) / 1000.0
        rospy.loginfo("Decoded AccX: %f, AccY: %f, AccZ: %f", acc_x, acc_y, acc_z)
        return acc_x, acc_y, acc_z
    elif data[7] == 0x02:
        gyro_x = (float)((int16_t)(data[1] << 8 | data[0])) / 100.0
        gyro_y = (float)((int16_t)(data[3] << 8 | data[2])) / 100.0
        gyro_z = (float)((int16_t)(data[5] << 8 | data[4])) / 100.0
        rospy.loginfo("Decoded GyroX: %f, GyroY: %f, GyroZ: %f", gyro_x, gyro_y, gyro_z)
        return gyro_x, gyro_y, gyro_z
    else:
        # Handle unknown data [7] value
        rospy.logwarn("Unknown data [7] value: %s", data[7])
        return None

'''
def can_callback(data):
    can_data = data.split('#')[1] # Assuming the data is in the format '585#xxxxxx'
    can_bytes = [int(can_data[i:i+2], 16) for i in range(0, len(can_data), 2)]

    decoded_values = decode_can_frame(can_bytes)
    if decoded_values is not None:
        roll, pitch, yaw = decoded_values
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.orientation.x = roll
        imu_msg.orientation.y = pitch
        imu_msg.orientation.z = yaw
        pub.publish(imu_msg)
'''

def main():
    rospy.init_node('can_decode_and_publish_mode', anonymous=True)

    can_interface = 'can0'

    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((can_interface,))

    pub = rospy.Publisher("/imu/data/raw", Imu, queue_size=10)

    while not rospy.is_shutdown():
        try:
            data = s.recv(16)
            #method1
            #can_id, can_dlc, can_data = struct.unpack('IBs', data)
            #can_bytes = list(can_data[:can_dlc])
            
            #method2
            ##can_id, can_dlc, can_data = struct.unpack('IB{}S'.format(can_dlc), data)
            ##can_bytes = list(can_data[:can_dlc])

            #method3
            #can_id, can_dlc = struct.unpack('IB', data[:5])  # Adjusted format specifiers
            #can_data = struct.unpack('{}s'.format(can_dlc), data[5:]) #Adjusted format specifiers
            #can_bytes = list(can_data[0])

            #method4
            #can_id, can_dlc = struct.unpack('IB', data[:5])  # Adjusted format specifiers
            #can_data = struct.unpack('{}B'.format(can_dlc), data[5:]) #Adjusted format specifiers
            #can_bytes = list(can_data)

            #method5
            #can_id, can_dlc = struct.unpack('IB', data[:5])
            #can_data = struct.unpack('{}B'.format(can_dlc), data[5:5+can_dlc])
            #can_bytes = list(can_data)

            #method6
            can_id, can_dlc = struct.unpack('IB', data[:5])
            can_data = struct.unpack('{}B'.format(can_dlc), data[5:5+can_dlc])
            can_bytes = list(can_data)


            # change %f to %d for integer
            #Change from %d to %u for unsigned int
            #change %X foe hexadecimal
            #method1
            #rospy.loginfo("Check : %X", can_id & (1 << 32)) #Ensure can_id is non_negative
            ##can_decode_and_publish_modeC
            #method2
            rospy.loginfo("Check : %X", can_id & 0xFFFFFFFF)

            if can_id == 0x585:
                decoded_values = decode_can_frame(can_bytes)                
                if decoded_values is not None:
                    roll, pitch, yaw = decoded_values
                    imu_msg = Imu()
                    imu_msg.header.stamp = rospy.Time.now()
                    imu_msg.orientation.x = roll
                    imu_msg.orientation.y = pitch
                    imu_msg.orientation.z = yaw
                    pub.publish(imu_msg)

        except socket.error as e:
        #Add handling error from socket
            rospy.logerr("Socket error: %s", str(e))
            #print the full traceback for more details
            rospy.logerr(traceback.format_exc)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
         