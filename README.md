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
    * Create new file on directory joystick_remapper/launch with a name "joystick_remapper_ps3_hadarobot.launch" as an example. After completed this step you will see new file called joystick_remapper_ps3_hadarobot.launch
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

  * Start bluetooth joystick node and joystick topic remapper 
  ```
  roslaunch joystick_remapper joystick_remapper_ps3_hadarobot.launch 
  ```
  * Start joystick controller safety node 
  ```
  rosrun cansend_generator joy_detect.py
  ```
  * Start CAN Bus Communication with HADA H500 Robot
    Option 1
    ```
    rosrun hada_bringup bringup_can2usb_hada.bash 
    ```
    Option 2 (Optionally to start CAN Bus communication could be done by this command without hada_bringup package)
    ```
    rosrun hada_bringup bringup_can2usb_hada.bash 
    ```
    Troubleshooting CAN Bus communication (You will see incoming CAN frame messages from HADA H500 robot)
    ```
    candump can0
    ```
  * Start ackerman drive steering control node 
  ```
  rosrun ackermann_drive_teleop joyop.py
  ```
  * Start sending CAN frame message to H500 Sprayer
  ```
  rosrun cansend_generator cansend_generator.py
  ```

* Check this video for [details](https://www.youtube.com/watch?v=bRNPGkcOvKI)
* Check this [figure](https://github.com/ditodamaru/cansend_ws_aero/blob/main/docs/rosgraph_hada_remote_control.png) for troubleshooting
* Check this ![figure](https://github.com/ditodamaru/cansend_ws_aero/blob/main/docs/rosgraph_hada_remote_control.png?raw=true) for troubleshooting


## Basic usage of the ROS packages for development


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











### Launch Script for joystick_remapper. Create new file on directory joystick_remapper/launch with name joystick_remapper_ps3_hadarobot.launch as an example 
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




# README.md Template 
# Project Title

Simple overview of use/purpose.

## Description

An in-depth paragraph about your project and overview of use.

## Getting Started

### Dependencies

* Describe any prerequisites, libraries, OS version, etc., needed before installing program.
* ex. Windows 10

### Installing

* How/where to download your program
* Any modifications needed to be made to files/folders

### Executing program

* How to run the program
* Step-by-step bullets
```
code blocks for commands
```

## Help

Any advise for common problems or issues.
```
command to run if program contains helper info
```

## Authors

Contributors names and contact info

ex. Dominique Pizzie  
ex. [@DomPizzie](https://twitter.com/dompizzie)

## Version History

* 0.2
    * Various bug fixes and optimizations
    * See [commit change]() or See [release history]()
* 0.1
    * Initial Release

## License

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details

## Acknowledgments

Inspiration, code snippets, etc.
* [Simple-readme](https://gist.github.com/DomPizzie/7a5ff55ffa9081f2de27c315f5018afc)
* [awesome-readme](https://github.com/matiassingers/awesome-readme)
* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2)
* [dbader](https://github.com/dbader/readme-template)
* [zenorocha](https://gist.github.com/zenorocha/4526327)
* [fvcproductions](https://gist.github.com/fvcproductions/1bfc2d4aecb01a834b46)