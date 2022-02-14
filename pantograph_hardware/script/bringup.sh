#!/bin/bash

echo "Restarting EtherCAT master services."
sudo /etc/init.d/ethercat stop
sudo /etc/init.d/ethercat start

roslaunch pantograph_hardware bringup.launch
