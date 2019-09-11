#!/bin/sh

sudo docker run --rm \
-it \
--net host \
-e DISPLAY=$DISPLAY \
-v "/$(pwd)/catkin_ws/:/catkin_ws/" \
-v $HOME/.Xauthority:/root/.Xauthority \
ros-nakaoka:latest
