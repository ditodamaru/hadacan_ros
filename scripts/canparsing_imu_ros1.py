#!/usr/bin/env python

#-- Author: Anditya Sridamar Pratyasta : anditya.sridamar.p@kangwon.ac.kr

import rospy
from sensor_msgs.msg import Imu
import socketcan
import struct

def decode_can_frame(data):
    if data[7] == 0x00:
        roll = struct.unpack('<h', bytes(data[0:2]))[0] / 100.0
        pitch = struct.unpack('<h', bytes(data[2:4]))[0] / 100.0
        yaw = struct.unpack('<h', bytes(data[4:6]))[0] / 100.0
        rospy.loginfo("Decoded Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw)
        return roll, pitch, yaw
    elif data[7] == 0x01:
        acc_x = struct.unpack('<h', bytes(data[0:2]))[0] / 1000.0
        acc_y = struct.unpack('<h', bytes(data[2:4]))[0] / 1000.0
        acc_z = struct.unpack('<h', bytes(data[4:6]))[0] / 1000.0
        rospy.loginfo("Decoded AccX: %f, AccY: %f, AccZ: %f", acc_x, acc_y, acc_z)
        return acc_x, acc_y, acc_z
    elif data[7] == 0x02:
        gyro_x = struct.unpack('<h', bytes(data[0:2]))[0] / 100.0
        gyro_y = struct.unpack('<h', bytes(data[2:4]))[0] / 100.0
        gyro_z = struct.unpack('<h', bytes(data[4:6]))[0] / 100.0
        rospy.loginfo("Decoded GyroX: %f, GyroY: %f, GyroZ: %f", gyro_x, gyro_y, gyro_z)
        return gyro_x, gyro_y, gyro_z
    else:
        # Handle unknown data[7] value
        rospy.logwarn("Unknown data[7] value: %s", data[7])
        return None

def main():
    rospy.init_node('can_decode_and_publish_node', anonymous=True)

    can_interface = 'vcan0'
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((can_interface,))

    pub = rospy.Publisher("/imu/data/raw", Imu, queue_size=10)

    while not rospy.is_shutdown():
        try:
            data = s.recv(16)
            #can_id, can_dlc, can_data = struct.unpack('IB8s', data)
            #can_bytes = list(can_data[:can_dlc])
            #decoded_values = decode_can_frame(can_bytes)

            #Method 6
            can_id, can_dlc = struct.unpack('IB', data[:5])

            # check if the the received cab ID matches the desired ID
            if can_id == 0x80000586:
                can_data= struct.unpack('{}B'.format(can_dlc), data[5:5 + can_dlc])
                can_bytes = list(can_data)
                decoded_values = decode_can_frame(can_bytes)

                if decoded_values is not None:
                    roll, pitch, yaw = decoded_values
                    imu_msg = Imu()
                    imu_msg.header.stamp = rospy.Time.now()
                    imu_msg.orientation.x = roll
                    imu_msg.orientation.y = pitch
                    imu_msg.orientation.z = yaw
                    pub.publish(imu_msg)
        except socket.error:
            pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
