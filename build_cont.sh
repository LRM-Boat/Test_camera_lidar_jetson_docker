#!/usr/bin/env bash


xhost +si:localuser:root
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}


docker run -it --rm \
  --name ximeacont \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/root/.Xauthority \
  -v ${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v /mnt/m2data/savior:/data:rw \
  -e QT_X11_NO_MITSHM=1 \
  --cap-add=NET_ADMIN \
  --cap-add=NET_RAW \
  --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /run/udev:/run/udev:ro \
  -e ROS_DOMAIN_ID=15 \
  --entrypoint /bin/bash \
  ros2-ximea-cli
