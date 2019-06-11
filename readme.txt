This is the source code for my Final Year Project which is about Collision avoidance on UAV using the Texas Instrument IWR1443 Radar.
Runs on ROS. Most of the code is a modified version of the TurtlBot example from the TI Resource website.
Hence, I have every package required to run, including the TurtleBot package. 
Some of these packages might be redundant and can be removed. 

There are several steps needed to install the software.
Assuming you already installed ROS Kinetic Full desktop version and using Ubuntu 16.04, you would have to:
  1. Install MAVROS package by following the instruction at https://dev.px4.io/en/ros/mavros_installation.html
  2. Make an empty catkin workspace folder
  3. Download the catkin workspace zip file from GitHub then copy the content of the zip into the workspace.
     Make sure that there is an “src” folder inside the catkin workspace.
  4. Build the catkin workspace using “catkin_make” command

To run the code for collision avoidance, type these 3 commands in order:
  1. roslaunch mavros px4.launch (start MAVROS and MAVLink communication)
  2. roslaunch uav_radar px4_tf.launch
  3. roslaunch turtlebot_mmwave_launchers radar_nav_uav1.launch
  
For mapping, change the last command to "roslaunch turtlebot_mmwave_launchers radar_mapping.launch"

To see the radar visulatization run RVIZ with follwing commands

- rosrun rviz rviz -d ~/rdr1/src/turtlebot_mmwave_launchers/launch/navigation_visualization.rviz
or
- rosrun rviz rviz -d ~/rdr1/src//turtlebot_mmwave_launchers/launch/mapping_visualization.rviz
