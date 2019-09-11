#!/bin/sh

docker run --rm -it     --net host     -e DISPLAY=$DISPLAY     -v ~/docker/docker-kobuki:/home     docker-kobuki
