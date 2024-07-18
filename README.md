# PoppyErgoJrROS

## About The Project

[![Project Screen Shot][project-screenshot]]()

This repository contains a series of code for controlling a Poppy Ergo Jr robot using the [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) to experiment with [ROS](https://www.ros.org/). 

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

* [![C++][cpp-shield]][cpp-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

To set up the project locally, you need to install (if not already the case) some dependencies.
To get a local copy up and running follow these steps.

### Prerequisites

* C++ Compiler

Install the build-essential package
  ```sh
  sudo apt install build-essential 
  ```
  
* Dynamixel SDK
  
 Download the Dynamixel SDK
 ```sh
 git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
 ```
 
 Go to the c++/build folder
  ```sh
 cd DynamixelSDK/c++/build/linux64
 ```

 Compile and install
  ```sh
 make
 sudo make install
 ```
 
 Upgrade USB access privileges by aading your account to the dialout group
   ```sh
  sudo usermod -aG dialout <your_account_id>
 ```
 
 * ROS
 
 Follow instructions provided [here](http://wiki.ros.org/Installation/Ubuntu).
 

### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/GuillaumeGibert/PoppyErgoJrROS.git
   ```
2. Open a terminal
3. Compile/Link by calling the makefile
 ```sh
   catkin-build
   source devel/setup.bash
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE EXAMPLES -->
## Usage

### ROS Master

1. Open a terminal
2. Launch the ROS Master
```sh
roscore
```

### Poppy controller

3. Open another terminal
4. Launch the robot controller
```sh
rosrun poppy_ros poppy_ros
```

### Inverse Kinematics server

5. Open another terminal
6. Launch the inverse kinematics server
```sh
rosrun poppy_ros ik_server
```

### Inverse Kinematics client

7. Open another terminal
8. Launch the camera calibration executable
```sh
rosrun poppy_ros ik_client x y z
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LICENSE -->
## License

Distributed under the GPL License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

Guillaume Gibert

Project Link: [https://github.com/GuillaumeGibert/PoppyErgoJrROS](https://github.com/GuillaumeGibert/PoppyErgoJrROS)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[arduino-shield]: https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white
[arduino-url]: https://www.arduino.cc/
[python-shield]: https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white
[python-url]: https://www.python.org/
[opencv-shield]: https://img.shields.io/badge/OpenCV-27338e?style=for-the-badge&logo=OpenCV&logoColor=white
[opencv-url]: https://opencv.org/
[cpp-shield]: https://img.shields.io/badge/-C++-blue?logo=cplusplus
[cpp-url]: https://isocpp.org/

[project-screenshot]: images/screenshot.png

[contributors-shield]: https://img.shields.io/github/contributors/GuillaumeGibert/PoppyErgoJrROS.svg?style=for-the-badge
[contributors-url]: https://github.com/GuillaumeGibert/PoppyErgoJrROS/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/GuillaumeGibert/PoppyErgoJrROS.svg?style=for-the-badge
[forks-url]: https://github.com/GuillaumeGibert/PoppyErgoJrROS/network/members
[stars-shield]: https://img.shields.io/github/stars/GuillaumeGibert/PoppyErgoJrROS.svg?style=for-the-badge
[stars-url]: https://github.com/GuillaumeGibert/PoppyErgoJrROS/stargazers
[issues-shield]: https://img.shields.io/github/issues/GuillaumeGibert/PoppyErgoJrROS.svg?style=for-the-badge
[issues-url]: https://github.com/GuillaumeGibert/PoppyErgoJrROS/issues
[license-shield]: https://img.shields.io/github/license/GuillaumeGibert/PoppyErgoJrROS.svg?style=for-the-badge
[license-url]: https://github.com/GuillaumeGibert/PoppyErgoJrROS/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/guillaume-gibert-06502ba4