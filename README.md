# Joystick Remote Control for HADA H500 Sprayer CANBUS-Based Protocol and software stack interface to ROS

This ROS package was tested under ROS 1 Melodic installed on Ubuntu 18.04 and Joytron MXSwitch Joystick version was used to test the functionality. Other joystick brand and version may need adjustment.   


## List To-DO

- [v] add functionality to check joystick connection on (cansend_generator.py) for safety
- [v] add block to give understanding this node controlled by joystick or external
- [v] rewrite joy_detect.py
- [ ] improving detection speed on joy_detect.py
- [v] add block to indicate joystick on standby state which indicate by there is no incoming message on axis
- [v] combining ('/joystick_control_state') to joy_connection_status
- [v] create connection between joy_detect.py <-> rosrun ackermann_drive_teleop joyop.py 

## Basic usage of the ROS packages
### Install dependencies

1. [Joystick Remapper](http://wiki.ros.org/joystick_remapper) Use this launch script 
2. [HADA bringup ROS Package]
3. Fork this branch (for developer)
    * Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
    * Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
    * Push to the Branch (`git push origin feature/AmazingFeature`)

4. 
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
  ```
  rosrun hada_bringup bringup_can2usb_hada.bash 
  ```
  * Start ackerman drive steering control node 
  ```
  rosrun ackermann_drive_teleop joyop.py
  ```

* Check this video for [details](https://www.youtube.com/watch?v=bRNPGkcOvKI)
* Check this [figure](https://github.com/ditodamaru/cansend_ws_aero/blob/main/docs/rosgraph_hada_remote_control.png) for troubleshooting




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



### Launch Script for joystick_remapper




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