#!/bin/bash

function frontcamera {
    while [ -x $0 ]; do
        mjpg_streamer -i "input_opencv.so --filter /home/ubuntu/Documents/vision/build/libkryptoncv-front.so -d /dev/video-front" -o "output_http.so -p 5801"
    done
}

function rearcamera {
    while [ -x $0 ]; do
        mjpg_streamer -i "input_opencv.so --filter /home/ubuntu/Documents/vision/build/libkryptoncv-back.so -d /dev/video-back" -o "output_http.so -p 5802"
    done
}

frontcamera &
rearcamera &
