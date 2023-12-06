# IsRoBot
Github repository for the catkin workspace of IsRoBot
Welcome to the IsRoBot project workspace! This workspace contains the ROS packages and configurations for the IsRoBot exhibition robot.

For the IsRoBot project, we've endeavoured to make it semi-autonomous, That means utilising Simulatneous Localisation and Mapping (SLAM), Custom Path planning algorithm, Sensor Fusion, Object Detection(For detecting QR code landmarks), and a Voice Assistant for question and answering. 

For your convenience, a small flowchart has been drawn below:

Camera and Lidar -> Sensor Fusion -> Mapping -> Custom Path Planning -> Motor control
Camera and Lidar -> Sensor Fusion -> Obstacle Avoidance (Or can be done only using Lidar data) -> Custom Path Planning -> Motor control
Imu and Encoders -> Sesnor Fusion -> Localisation -> Custom Path Planning -> Motor control
camera -> Object Detection (for determining QR Landmarks) -> Depth Estimation (For determining distance to landmarks) -> Custom Path Planning -> Motor Control

OS used: Linux (Ubuntu 20.04 (Focal) or higher)
Ros version used: ROS Noetic Ninjemys

Hardware: Please see the image attached to get an idea about which sensors are connected to which microprocessor/microcontroller
Microporcessor to be used: NVIDIA Jetson Nano B01
Microcontroller to be used: Arduino MEGA
NOTE: We planned to use an Arduino as a slave so as to decrease the processing load on Jetson Nano. It is not recommended, but you may choose to not use an Arduino
(You will have to use the rosserial library to connect Arduino to Jetson nano as a slave)
If you choose to use an Arduino, you will host only the following devices:
5 ultrasonic sensors (for cliff detection)
2 motor controllers
4 encoders

Rest of the sensors/devices shall be connected to NVIDIA Jetson Nano (You may choose to add a temperature sensor for doing predictive maintenance for the arduino and jetson nano so as to avoid overheating problems)
Only Imu shall be connected to the digital pins of the microprocessor, rest of the sensors will be connected via USB or other ports.
To determine which device is connected to which port, kindly refer to the Appendix section of the Project Report for IsRoBot.

Getting Started
Create your catkin workspace:
After installing ROS noetic to your system, create a folder for housing the catkin workspace
mkdir --parents <name_of_your_workspace>

Clone the Repository:
git clone https://github.com/Renee196/IsRoBot.git -b master

Navigate to the Workspace:
cd <name_of_your_workspace>

In the workspace, follow the below 3 steps:
1. Initialize Catkin Workspace:
catkin init

2. Build the Workspace:
catkin_make

3. Source the Workspace:
source devel/setup.bash

Now your workspace is set up and ready for use.

Next Steps
NOTE: For an overview of the functionality of ROS in IsRoBot, you can refer to the project report. For convenience, the ROS Architecture has also been added to this README file.

Review Package Descriptions:
Explore the packages in the src directory to understand their functionalities.

Configure Hardware Drivers: (You can refer to the Appendix of Pranavi Arora's project report. It will give you a basic idea. You may have to research more to get it right)
Configure and launch drivers for your robot's hardware (sensors, actuators, etc.).

Set Up Simulation Environment:
If applicable, set up a simulation environment for testing.
The URDF of IsRoBot has already been developed. However, the wheel movements might give trouble. Change the URDF as needed. You may also change the topics to which sensor plugins publish the data
You will have to create your own Gazebo world mimicking the exhibition space.
To spawn the URDF in a gazebo world, use the following command:
rosrun gazebo_ros spawn_model -file $(rospack find hardware_interface_pkg)/urdf/isrobot.urdf -urdf -x 0 -y 0 -z 0 -model isrobot
To launch an empty world:
roslaunch gazebo_ros empty_world.launch

For convenience, you can create a launch file for the same. It will look something like this:
<launch>
    <!-- Specify the path to your URDF file -->
    <arg name="urdf_file" default="$(find hardware_interface_pkg)/urdf/isrobot.urdf" />

    <!-- Specify the model name -->
    <arg name="model_name" default="isrobot" />

    <!-- Launch Gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch" /> <!--or your custom gazebo world-->

    <!-- Spawn the URDF model -->
    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" output="screen"
          args="-file $(arg urdf_file) -urdf -model $(arg model_name)">
    </node>
</launch>

NOTE: If you encounter a svga_destroy error after launching a gazebo world, add the following to the bash script:
echo "export SVGA_VGPU10=0" >> ~/.bashrc

Explore Launch Files:
Check the launch directory for launch files. Modify or create new launch files as needed.

Run ROS Nodes:
Start ROS nodes using launch files or individually based on your requirements. Modify or create new nodes as needed.
The codes are working individually, but since we have added rospy to the basic code and has not been tested, it is advised to test the functionality of each node individually, using real sensors or simulated ones.

Customize and Extend:
Customize existing packages or create new ones to extend the functionality of IsRoBot.

If you encounter issues or have questions, feel free to contact us.

Make sure to follow best practices for code development and ROS package structuring.

Here's a brief description for each package in your isrobot project:

control_pkg:
Contents: Contains scripts for controlling the robot, including gesture control (gesture_control.py), remote control (remote_control.py), and voice control (voice_control.py).
Purpose: Enables various methods of controlling the robot, allowing for flexibility in user interaction.

custom_msgs:
Contents: Defines custom ROS messages (e.g., MoveMotor.msg, PathUpdate.msg, QRCodeInfo.msg, QRDetectionInfo.msg, QRDistanceInfo.msg) used for communication between different parts of the robot.
Purpose: Facilitates communication by defining message structures tailored to the specific needs of the robot.

gazebo_ros_pkgs:
Contents: Contains standalone Gazebo ROS packages with plugins and tools for simulating sensors, cameras, and other robot components in the Gazebo simulation environment.
Purpose: Essential for Gazebo simulation, allowing developers to test and validate the robot's behavior in a controlled environment.

hardware_interface_pkg:
Contents: Includes configuration files, scripts, and URDF files related to the hardware interface of the robot. Meshes (frustum.dae and midbody.dae) and URDF file (isrobot.urdf) define the physical structure of the robot.
Purpose: Defines the hardware aspects of the robot, necessary for accurate simulation and interaction with the physical world

joint_state_publisher:
Contents: Provides a node (joint_state_publisher) and configuration files for publishing joint states of the robot. The github repo has been modified in the isrobot workspace. Kindly check it out before proceeding to use the joint_state_publisher for gazebo simulation.
Purpose: Necessary for visualizing the robot's state accurately in ROS tools like RViz and sim tools like Gazebo.

navigation_pkg:
Contents: Contains configuration files and scripts related to robot navigation, including path planning and obstacle avoidance. The obstacle avoidance code has not been updated, since it is contained in the path planning node. Feel free to modify the nodes of any package according to yourself.
Purpose: Enables the robot to navigate autonomously in its environment, avoiding obstacles and following predefined paths.

perception_pkg:
Contents: Includes scripts for perception tasks such as cliff detection (cliff_detection.py), depth estimation (depth_estimation.py), sensor fusion (ekf_sensor_fusion.py), object detection (object_detection.py), obstacle avoidance (obstacle_avoidance.py), path planning (path_planning.py), and QR code scanning (scan_qr.py).
Purpose: Enhances the robot's perception capabilities, crucial for navigation, interaction, and understanding its surroundings.

robot_localization:
Contents: A standalone package for sensor fusion and state estimation, providing tools for integrating data from multiple sensors.
Purpose: Improves the accuracy of robot localization by fusing information from sensors such as encoders and IMU.

teleop_twist_keyboard:
Contents: Includes a ROS node (teleop_twist_keyboard.py) for teleoperating the robot using keyboard commands.
Purpose: Facilitates manual control of the robot during testing and development.

user_interface_pkg:
Contents: Placeholder for user interface-related files. Has not been updated yet, but for deployment of the robot, this package will be important
Purpose: Reserved for potential future development of a user interface for interacting with the robot.

utilities_pkg:
Contents: Placeholder for utility scripts and configuration files.
Purpose: Reserved for potential future utility scripts or configurations.

voice_assistant_pkg:
Contents: Contains a ROS node (voice_assistant.py) for enabling voice interaction with the robot.
Purpose: Provides a natural and intuitive way for users to communicate with the robot using voice commands.

SLAM has to be implemented using the gmapping standalone package.

Have fun exploring and extending the capabilities of IsRoBot!

![IsRoBot ROS Architecture](https://github.com/Renee196/IsRoBot/assets/76276943/45ae278a-2874-47da-9622-8cc6b9577670)

![image](https://github.com/Renee196/IsRoBot/assets/76276943/da3455d8-28e7-4ca6-930e-49fb40674d37)
