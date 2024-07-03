#!/usr/bin/env python

__author__ = 'Anditya Sridamar Pratyasta'
<<<<<<< HEAD
__license__ = 'GPLv3'
__maintainer__ = 'Anditya Sridamar Pratyasta'
=======
>>>>>>> 150ac82e85174dd8aa59afb799c78c3e2dd019bf
__email__ = 'anditya.sridamar.p@kangwon.ac.kr'

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32  # Import Int32 message type
from rospy.timer import TimerEvent

class JoystickConnectionDetector:
    def __init__(self):
        rospy.init_node('joystick_connection_detector', anonymous=True)
        self.last_msg_time = rospy.get_rostime()
        self.connected = True
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)
        self.error_publisher = rospy.Publisher('/joy_connection_error', Int32, queue_size=10)  # Publish Int32 messages        
        self.control_state_publisher = rospy.Publisher('/joy_control_state', Int32, queue_size=10)  # Publish control state


    def joy_callback(self, msg):
        self.last_msg_time = rospy.get_rostime()
        if not self.connected:
            rospy.loginfo("Joystick connection restored!")
            self.connected = True
            self.error_publisher.publish(1)  # Publish 1 to indicate connection restored

        if msg.buttons[6] == 1:  # Check if button column 6 is pressed
            self.control_state_publisher.publish(3)  # Publish 3 to indicate joystick in control state
            rospy.loginfo("Joystick on control state!")
        else:
            self.control_state_publisher.publish(4)  # Publish 0 to indicate joystick not in control state 
            rospy.loginfo("Joystick on stanby state!")           


    def timer_callback(self, event):
        current_time = rospy.get_rostime()
        if (current_time - self.last_msg_time).to_sec() > 1.0:
            if self.connected:
                rospy.logerr("Joystick connection lost!")
                self.connected = False
            # Do something when joystick connection is lost
                self.error_publisher.publish(0)  # Publish 0 to indicate connection lost


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = JoystickConnectionDetector()
    detector.run()
