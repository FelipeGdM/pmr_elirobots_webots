#!/bin/bash

docker build . -t elibots
docker run -p 1234:1234 --rm \
    --gpus=all \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    --name elibots_webots \
    elibots
