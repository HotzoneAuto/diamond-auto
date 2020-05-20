#!/usr/bin/env bash

USER_ID=$(id -u)
DOCKER_USER=apollo

if [[ "$USER" != "apollo" ]] || [[ $USER_ID -ne 1000 ]]; then
    DOCKER_USER=$USER
fi

xhost +local:root 1>/dev/null 2>&1
docker exec \
    -u $DOCKER_USER \
    -it geek_dev_$USER \
    /bin/bash
xhost -local:root 1>/dev/null 2>&1
