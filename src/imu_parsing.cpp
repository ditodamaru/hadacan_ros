//
// Author: Anditya Sridamar Pratyasta : anditya.sridamar.p@kangwon.ac.kr
//

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>

void decode_can_frame(const can_frame& frame, ros::Publisher& imu_pub, ros::Publisher& wheel_speed_pub) {
    if (frame.can_id == 0x80000586) {
        /*
        if (frame.data[7] == 0x00) {
            int16_t roll = *reinterpret_cast<const int16_t*>(&frame.data[0]) / 100.0;
            int16_t pitch = *reinterpret_cast<const int16_t*>(&frame.data[2]) / 100.0;
            int16_t yaw = *reinterpret_cast<const int16_t*>(&frame.data[4]) / 100.0;

            ROS_INFO("Decoded Roll: %f, pitch: %f, Yaw: %f", roll, pitch, yaw);

            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.orientation.x = roll;
            imu_msg.orientation.y = pitch;
            imu_msg.orientation.z = yaw;
            imu_msg.orientation.w = 0.0;  // Quaternion orientation, w component
            imu_pub.publish(imu_msg);
        } else if (frame.data[7] == 0x01) {
            int16_t acc_x = *reinterpret_cast<const int16_t*>(&frame.data[0]) / 1000.0;
            int16_t acc_y = *reinterpret_cast<const int16_t*>(&frame.data[2]) / 1000.0;
            int16_t acc_z = *reinterpret_cast<const int16_t*>(&frame.data[4]) / 1000.0;

            ROS_INFO("Decoded AccX: %f, AccY: %f, AccZ: %f", acc_x, acc_y, acc_z);

            // Add logic to handle accelerometer data

        } else if (frame.data[7] == 0x02) {
            int16_t gyro_x = *reinterpret_cast<const int16_t*>(&frame.data[0]) / 100.0;
            int16_t gyro_y = *reinterpret_cast<const int16_t*>(&frame.data[2]) / 100.0;
            int16_t gyro_z = *reinterpret_cast<const int16_t*>(&frame.data[4]) / 100.0;

            ROS_INFO("Decoded GyroX: %f, GyroY: %f, GyroZ: %f", gyro_x, gyro_y, gyro_z);
            
            // Add logic to handle gyroscope data

        } else {
            // Handle unknows data [7] value
            ROS_WARN("Unknown data[7] value: %d", frame.data[7]);
        }
        */
    } else if (frame.can_id == 0x80000582) {
        // Handle wheel speed data
        int32_t WheelEncoder = frame.data[4];
        WheelEncoder += frame.data[5] << 8;
        WheelEncoder += frame.data[6] << 16;
        WheelEncoder += frame.data[7] << 24;
        
        if (WheelEncoder > 0x7FFF) WheelEncoder = (0xFFFF - WheelEncoder);
            else WheelEncoder = - WheelEncoder;

        double RPM = (WheelEncoder / 5.0 * 10.0);
        double M_S = (WheelEncoder * 3.14 * 0.281) / (5.0 * 10 * 30.0);

        ROS_INFO("Decoded Wheel Speed: RPM = %f, M/S = %f", RPM, M_S);

        // Create and publish a message using geometry_msgs::twist
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = M_S;

        wheel_speed_pub.publish(twist_msg);
    } else {
        // Handle other CAN IDs if needed
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_decode_and_publish_node");
    ros::NodeHandle nh;

    const char* can_interface = "can0";
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s == -1) {
        perror("Error opening socket");
        return EXIT_FAILURE;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;

    strcpy(ifr.ifr_name, can_interface);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("Error binding socket");
        close(s);
        return EXIT_FAILURE;
    }


    struct can_frame frame;

    //Load IMU frame ID parameter
    //nh.param<std::string>("imu_frame_id", imu_frame_id, "imu_link")

    // Create a ROS publisher
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data/raw", 10);
    ros::Publisher wheel_speed_pub = nh.advertise<geometry_msgs::Twist>("/sensor_velocity", 10);

    while (ros::ok()) {
        ssize_t nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Error reading from socket");
            close(s);
            return EXIT_FAILURE;
        }    

        if (nbytes < sizeof(struct can_frame)) {
            std::cerr << "Incomplete CAN frame received\n";
            continue;
        }

        decode_can_frame(frame, imu_pub, wheel_speed_pub);
        ros::spinOnce();
    }

    close(s);
    // return EXIT_SUCCESS;
    return 0;
}
