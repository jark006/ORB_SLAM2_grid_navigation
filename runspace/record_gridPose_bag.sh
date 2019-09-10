#!/usr/bin/env bash

date_time="$(date +%F-%T)"
location="office-509"
sensor="gridPose"
output_path="./bags"
filename="${location}-${sensor}-${date_time}.bag"

rosbag record -O ${output_path}/${filename} pts_and_pose all_kf_and_pts /cur_camera_pose
