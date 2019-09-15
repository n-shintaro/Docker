#!/bin/sh

docker run --rm \
-it \
--net host \
-e DISPLAY=$DISPLAY \
-v "/$(pwd)/catkin_ws/:/catkin_ws/" \
--name ros-nakaoka \
-v $HOME/.Xauthority:/root/.Xauthority \
ros-nakaoka:latest
