#!/usr/bin/bash

# Custom function to source ROS setup file
ros_source(){
    if [ -f $1 ]; then
        source $1
    else
        echo "Can't find ROS setup.bash: " $1
    fi
}

# Find full path to Wanda repo directory
wanda_path=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Source most of the ROS code
ros_setup=$wanda_path/ROS/devel/setup.bash
ros_source $ros_setup

# Source the SLAM-specific stuff separately
slam_setup=$wanda_path/ORB_SLAM/ROS_SLAM/devel/setup.bash
ros_source $slam_setup

#================================================
# ROS distributed processing stuff
#================================================
reset_rosmaster(){
    # This command tells ROS to use this machine
    # as the ROS core master
    export ROS_MASTER_URI=http://localhost:11311
}

# Set up ROS to use this machine as default
reset_rosmaster

# Function to connect to a ROS master on another machine.
# 1 argument given, with the IP-address of the desired
# machine running the ROS core
connect_ros(){
    if [[ $# -eq 0 ]]; then
        # No argument were given. Let the user know
        echo "Please supply IP address of desired ROS master machine"
    else
        # Connect to the ROS master machine from given IP address
        ros_master_ip=$1
        export ROS_MASTER_URI=http://$ros_master_ip:11311

    fi
}
