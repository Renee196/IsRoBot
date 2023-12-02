# IsRoBot
Github repository for the catkin workspace of IsRoBot
Welcome to the IsRoBot project workspace! This workspace contains the ROS packages and configurations for the IsRoBot exhibition robot.

Getting Started
Create your catkin workspace:
After installing ROS noetic to your system, create a folder for housing the catkin workspace
mkdir --parents <name_of_your_workspace>

Clone the Repository:
git clone https://github.com/Renee196/IsRoBot.git -b my_branch

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

Have fun exploring and extending the capabilities of IsRoBot!


