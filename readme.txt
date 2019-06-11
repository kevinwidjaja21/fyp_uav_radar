This is the source code for my Final Year Project which is about Collision avoidance on UAV using the Texas Instrument IWR1443 Radar.
Runs on ROS. Most of the code is a modified version of the TurtlBot example from the TI Resource website.
Hence, I have every package required to run, including the TurtleBot package. 
Some of these packages might be redundant and can be removed. 

There are several steps needed to install the software.
Assuming you already installed ROS Kinetic Full desktop version and using Ubuntu 16.04, you would have to:
  1. Install MAVROS package by following the instruction here
  2. Make an empty catkin workspace folder
  3. Download the catkin workspace zip file from GitHub then copy the content of the zip into the workspace.
     Make sure that there is an “src” folder inside the catkin workspace.
  4. Build the catkin workspace using “catkin_make” command

To run the code
