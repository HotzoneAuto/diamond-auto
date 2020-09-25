#!/usr/bin/env bash

# Usage:
#   ./build_geek_pcl.sh geek.Dockerfile
# Commit 
# docker commit 5a69f1c7051c geekstyle/geek_lite:geek_lite-${ARCH}-18.04-${TIME}

REPO=hotzoneauto2020/diamond-auto
ARCH=$(uname -m)
TIME=$(date +%Y%m%d_%H%M)

TAG="${REPO}:diamond-auto-${ARCH}-18.04-${TIME}"

CONTAINER_ID=$(docker ps | grep apollo_cyber_${USER}| awk '{print $1}')

docker commit "$CONTAINER_ID" "$TAG"
# docker tag "$TAG" "$RELEASE_NAME"
#docker stop "$CONTAINER_ID"

# Please provide credential if you want to login automatically.
#DOCKER_USER="mickeyouyou"
#DOCKER_PASSWORD="1Q2W3e4r"
#if [ ! -z "${DOCKER_PASSWORD}" ]; then
#  docker login -u ${DOCKER_USER} -p ${DOCKER_PASSWORD} ${DOCKER_REGISTRY}
#fi

#docker push ${TAG}
