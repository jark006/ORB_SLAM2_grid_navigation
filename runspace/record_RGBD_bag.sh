#!/usr/bin/env bash

date_time="$(date +%F-%T)"
location="office-509"
sensor="kinect"
output_path="./bags"
filename="${location}-${sensor}-${date_time}.bag"

rosbag record -O ${output_path}/${filename} /camera/rgb/image_raw /camera/depth_registered/image_raw
