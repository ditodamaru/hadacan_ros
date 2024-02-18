#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
import subprocess

# Constants
OFFSET1 = 30
OFFSET2 = 1000

def radian_to_degrees(radian):
    return radian * 180 / 3.14

def decimal_to_hexa(decimal):
    return hex(decimal)[2:].zfill(2)

def generate_cansend_frame(steering_angle, wheel_speed):
    payload = [0] * 8

    payload[0] = int(steering_angle + OFFSET1)
    payload[1] = int(wheel_speed + OFFSET2) & 0xFF
    payload[2] = int(wheel_speed + OFFSET2) >> 8

    payload_hexadecimal = [decimal_to_hexa(value) for value in payload]

    cansend_frame = "601#"
    cansend_frame += '.'.join(payload_hexadecimal)

    return cansend_frame.upper()

def send_cansend_frame(cansend_frame):
    try:
        subprocess.run(['cansend', 'can0', cansend_frame])
    except Exception as e:
        rospy.logerr("Error sending cansend frame: %s", str(e))

def ackermann_callback(data):
    steering_angle_rad = data.drive.steering_angle
    steering_angle_deg = radian_to_degrees(steering_angle_rad)
    wheel_speed_input = data.drive.speed

    # Calculate wheel_speed_value using the provided formula
    wheel_speed_value = (wheel_speed_input * (5 * 10 * 30)) / (3.14 * 0.281)

    cansend_frame = generate_cansend_frame(steering_angle_deg, wheel_speed_value)
    rospy.loginfo("Generated cansend frame: %s", cansend_frame)

    send_cansend_frame(cansend_frame)

def main():
    rospy.init_node('cansend_generator_node', anonymous=True)

    rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, ackermann_callback) #ackermann_steering_topic

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
