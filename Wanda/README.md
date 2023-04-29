# Wanda

This repository contains code, models, and documentation, for Wanda, the hexapod robot. This robot has been used as a project platform for technical electives during a masters in robotics program at Worcester Polytechnic Institute.

In order to run this ROS directory, the requirements are:
Ubuntu 20.04
ROS Noetic

Additionally, this directory is dependent on a few packages that need to be manually installed prior to build and launch:

``sudo apt install ros-noetic-geographic-msgs``
``sudo apt install libgeographic-dev``
``sudo apt install rviz``

For info on installing ROS Noetic and setup of a workspace directory, see: http://wiki.ros.org/noetic/Installation/Ubuntu

After cloning this repository, navigate to the "./ROS" folder within and execute the following commands:

``catkin_make`` (standard command to build)

``source devel/setup.bash`` (standard command to enable ROS in terminal)

Once this is done, verify installation with an rviz simulation:
In one terminal:
``roslaunch analysis run_all.launch``

Then, in another terminal:
``rviz -d wanda.rviz``

Recall that ``source devel/setup.bash`` must be executed in every terminal using ROS, not just initial setup.
