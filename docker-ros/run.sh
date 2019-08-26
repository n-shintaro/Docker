#!/bin/sh

sudo docker run --rm -it     --net host     -e DISPLAY=$DISPLAY     -v ~/docker/docker-ros:/home     ros
