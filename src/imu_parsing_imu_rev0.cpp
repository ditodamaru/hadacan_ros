//
// Author: Anditya Sridamar Pratyasta : anditya.sridamar.p@kangwon.ac.kr
//

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
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

void decode_can_frame(const can_frame& frame, ros::Publisher& pub) {
    if (frame.can_id == 0x80000586){
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
            pub.publish(imu_msg);
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
        
    } else if {
        // Handle other CAN IDs
    } else {
        // Handle other CAN IDs if needed
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_decode_and_publish_node");
    ros::NodeHandle nh;

    const char* can_interface = "vcan0";
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

    // Create a ROS publisher
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("Imu/data/raw", 10);

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

        decode_can_frame(frame, pub);
        ros::spinOnce();
    }

    close(s);
    // return EXIT_SUCCESS;
    return 0;
}
