#!/bin/bash

#roscore &

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:../Examples/ROS

rosrun ORB_SLAM2 RGBDpub ../Vocabulary/ORBvoc.bin ./kinect507.yaml

