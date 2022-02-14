#!/bin/bash

echo "Restarting EtherCAT master services."
sudo /etc/init.d/ethercat stop
sudo /etc/init.d/ethercat start

source "/home/rcluser/Projects/ros/ethercat_ws/devel/setup.bash"
roslaunch pantograph_hardware bringup.launch
