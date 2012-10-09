#!/bin/bash

# Run this as root, from the directory containing it!
#
# USAGE: sudo ./install.bash
#

stackPath=./

export robot=husky
export user=administrator
export release=$(ls /opt/ros/ | tail -n1)

source helpers.bash

rm -f /etc/ros/setup.bash
ln -s /home/${user}/ros/setup.bash /etc/ros/setup.bash
source /etc/ros/setup.bash
pushd `rospack find ${robot}_bringup`/upstart > /dev/null

install_udev_rules
install_job core eth0 11311

popd > /dev/null
