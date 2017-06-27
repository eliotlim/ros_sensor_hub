#!/bin/bash
# make_libraries.sh
# Purpose: Automatically generate rosserial msg/srv header
# files and copy them to the ~/Arduino/libraries folder
#
# @author Eliot Lim (github: @eliotlim)
# @version 1.0 (27/6/17)

echo "Generating libraries..."
rm -rf ros_lib
rosrun rosserial_client make_libraries .
echo "Copy libraries to ~/Arduino/libraries/Rosserial_Arduino_Library/src"
cp -R ros_lib/sensor_hub ~/Arduino/libraries/Rosserial_Arduino_Library/src
echo "Cleaning up..."
rm -rf ros_lib
echo "done."
