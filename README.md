# Joystick Remote Control for HADA H500 Sprayer CANBUS-Based Protocol and Software Stack Interface to ROS

This ROS package was tested under ROS 1 Melodic installed on Ubuntu 18.04 and Joytron MXSwitch Bluetooth Joystick version was used to test the functionality. Other joystick brand and version may need adjustment.   


## List To-Do

- [v] add functionality to check joystick connection on (cansend_generator.py) for safety
- [v] add block to give understanding this node controlled by joystick or external
- [v] rewrite joy_detect.py
- [ ] improving detection speed on joy_detect.py
- [v] add block to indicate joystick on standby state which indicate by there is no incoming message on axis
- [v] combining ('/joystick_control_state') to joy_connection_status
- [v] create connection between joy_detect.py <-> rosrun ackermann_drive_teleop joyop.py 

## Basic usage of the ROS packages
### Install dependencies (These step assume that the user will test the package under ROS 1 Melodic)

1. Build from source [Joystick Remapper ROS Package](http://wiki.ros.org/joystick_remapper) 
    * Create new ROS workspace environment (recommended) for ROS Melodic. For ROS Noetic may need to modify the python script on [joystick_remapper/script/joystick_remapper.py](https://github.com/epan-utbm/joystick_remapper/blob/melodic/scripts/joystick_remapper.py) to python3.
    ```
    mkdir -p ~/github/joystick_remapper_ws/src
    cd ~/github/joystick_remapper_ws
    catkin_make
    cd src
    git clone https://github.com/epan-utbm/joystick_remapper.git
    cd ..
    catkin_make
    source ~/github/joystick_remapper_ws/devel/setup.bash
    echo "source ~/github/joystick_remapper_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    * Create new file on directory joystick_remapper/launch with a name "joystick_remapper_ps3_hadarobot.launch" as an example. 
    
    After completed this step you will see new file called joystick_remapper_ps3_hadarobot.launch
    ```
    cd ~/github/joystick_remapper_ws/src/joystick_remapper/launch
    touch joystick_remapper_ps3_hadarobot.launch
    ```
    * Edit joystick_remapper_ps3_hadarobot.launch using Visual Studio Code and then copy this launch script below:
    ```
    <?xml version="1.0"?>
    <launch>
      <node pkg="joy" type="joy_node" name="ps3_joy" >
        <remap from="joy" to="ps3_joy" />
        <param name="dev" value="/dev/input/js0" />
        <param name="deadzone" value="0.1" />
        <param name="autorepeat_rate" value="10" />
      </node>
      <node pkg="joystick_remapper" type="joystick_remapper.py" name="ps3_to_hadarobot" >
        <remap from="joy_source" to="ps3_joy" />
        <remap from="joy_dest" to="joy" />
        <param name="button_mapping" type="str" value="=" />
        <param name="axis_mapping" type="str" value="0 3 1 2 4 5 6" /> <!-- result1 = 0 2 1 3 4 5 6-->
      </node>
    </launch>
    ```
2. Build from source [HADA bringup ROS Package](https://github.com/ditodamaru/hada_bringup)
    
    Recommended steps. Assumed that github directory already available.
    ```
    cd ~/github
    mkdir -p hada_bringup_ws/src
    catkin_make
    cd src
    git clone git@github.com:ditodamaru/hada_bringup.git
    cd ..
    catkin_make
    source ~/github/hada_bringup_ws/devel/setup.bash
    echo "source ~/github/hada_bringup_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
3. Build from source [Ackerman Drive Message Generator Package](https://github.com/ditodamaru/ackermann-drive-teleop)
    
    Recommended steps. Assumed that github directory already exist.
    ```
    cd ~/github
    mkdir -p ackerman_teleop_ws/src
    catkin_make
    cd src
    git clone -b melodic-devel git@github.com:ditodamaru/ackermann-drive-teleop.git
    cd ..
    catkin_make
    source ~/github/ackerman_teleop_ws/devel/setup.bash
    echo "source ~/github/ackerman_teleop_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

4. Build from source [this](https://github.com/ditodamaru/hadacan_ros) repository
    
    Recommended steps. Assumed that github directory already exist
    ```
    cd ~/github
    mkdir -p hadacan_ros_ws/src
    catkin_make
    cd src
    git clone git@github.com:ditodamaru/hadacan_ros.git
    cd ..
    catkin_make
    source ~/github/hadacan_ros_ws/devel/setup.bash
    echo "source ~/github/hadacan_ros_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

5. Launch ROS Nodes

  * Terminal 1: Start bluetooth joystick node and joystick topic remapper 
  ```
  roslaunch joystick_remapper joystick_remapper_ps3_hadarobot.launch 
  ```
  * Terminal 2: Start joystick controller safety node 
  ```
  rosrun cansend_generator joy_detect.py
  ```
  * Terminal 3: Start CAN Bus Communication with HADA H500 Robot

    For first installation
    ```
    rosrun hada_bringup setup_can2usb_hada.bash 
    ```
    After installation finished, directly use this steps below.
      
      Terminal 3: Option 1
      ```
      rosrun hada_bringup bringup_can2usb_hada.bash 
      ```
      Option 2 (Optionally to start CAN Bus communication could be done by this command without hada_bringup package)
      ```
      sudo ip link set can0 up type can bitrate 125000
      ```
      Terminal 4: Troubleshooting CAN Bus communication (You will see incoming CAN frame messages from HADA H500 robot)
      ```
      candump can0
      ```
  * Terminal 5: Start ackerman drive steering control node 
  ```
  rosrun ackermann_drive_teleop joyop.py
  ```
  * Terminal 6: Start sending CAN frame message to H500 Sprayer.

  **CAUTION:** Make sure to move the HADA H500 sprayer robot in a safe space.
  ```
  rosrun cansend_generator cansend_generator.py
  ```

* Check this video for [details](https://www.youtube.com/watch?v=bRNPGkcOvKI)
* Check this [figure](https://github.com/ditodamaru/cansend_ws_aero/blob/main/docs/rosgraph_hada_remote_control.png) for troubleshooting
* Check this figure below for troubleshooting![figure](https://github.com/ditodamaru/cansend_ws_aero/blob/main/docs/rosgraph_hada_remote_control.png?raw=true)


## Basic usage of the ROS packages for development
This guideline is intended to make integration between basic usage mentioned above and real-time robot position visualization purpose in RVIZ, also to do mininum requirement for making Visual SLAM (VSLAM) working based on RTABMap for **Handheld Mapping**.

1. Make sure to succesfully build from source [hada_explorer_ws](https://github.com/ditodamaru/hada_explorer_ws) package

2. Launch ROS Nodes

  * Terminal 7: Start visualize the H500 Sprayer in RVIZ 
  * a. (Make sure you have similar [H500 URDF xacro file equpped with RealSense D435i and GPS](https://github.com/ditodamaru/hada_explorer_ws/blob/master/h500_description/h500_sprayer/h500_ori_base_link_d435i_base_footprint.urdf.xacro)). 
  * b. And then copy and modify [this](https://github.com/ditodamaru/hada_explorer_ws/blob/master/hada_explorer_bot/hada_explorer_description/launch/hada_base_bringup_method2.launch) launch file according to your URDF file setting. **IMPORTANT** Make sure to change the position of gps_link and camera_link to base_link according to your depth camera position setting in reality. So this URDF file basically acting as a twin of your real robot.
  * c. (This command below is just an example if you want to test and learn about the H500 URDF xacro file equpped with RealSense D435i and GPS).
  * d. This RVIZ visualization will automatically subscribe to odometry message which will represent the robot movement in reality. How to get odometry message if the robot doesn't have any odometry message? We will explaine later. 
  ```
  roslaunch hada_explorer_description hada_base_bringup_method2.launch 
  ```
  - After above command executed, try to learn about TF (transform) tree of this H500 URDF xacro file with this command
  ```
  rosrun rqt_tf_tree rqt_tf_tree 
  ```
  * Terminal 8: Start the DepthAI OAK-D RGB-D Mode for **Handheld Mapping**:
  Read more about RGB-D Handheld Mapping for OAK-D camera [here](http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping)
  ```
  roslaunch depthai_examples stereo_inertial_node.launch
  ```
  * Terminal 9: Start to estimate quaternion of the IMU data
  ```
  rosrun imu_filter_madgwick imu_filter_node \
    imu/data_raw:=/stereo_inertial_publisher/imu \
    imu/data:=/stereo_inertial_publisher/imu/data  \
    _use_mag:=false \
    _publish_tf:=false
  ```

  * Terminal 10: Start Mapping Mode
  ```
  roslaunch rtabmap_launch rtabmap.launch \
    args:="--delete_db_on_start" \
    rgb_topic:=/stereo_inertial_publisher/color/image \
    depth_topic:=/stereo_inertial_publisher/stereo/depth \
    camera_info_topic:=/stereo_inertial_publisher/color/camera_info \
    imu_topic:=/stereo_inertial_publisher/imu/data \
    frame_id:=oak-d_frame \
    approx_sync:=true \
    wait_imu_to_init:=true
  ```
  * Terminal 11: Start Localization Mode
  - After the mapping process completed, then switch to localization mode or switch back to mapping mode if you are not satisfy with the map result.
  ```

  rosservice call rtabmap/set_mode_localization
  rosservice call rtabmap/set_mode_mapping
  ```

## Advance usage of this sofware stack for stereo outdoor navigation
**Read more in [Stereo Outdoor Navigation tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/StereoOutdoorNavigation)**


<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Authors

Contributors names and contact info
1. [Anditya Sridamar Pratyasta](https://www.linkedin.com/in/andidamar/)

