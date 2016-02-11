#!/bin/bash

function frontcamera {
    while [ -x $0 ]; do
        mjpg_streamer -i "input_opencv.so --filter /home/ubuntu/Documents/vision/build/libkryptoncv-front.so -d /dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0" -o "output_http.so -p 5801"
    done
}

function rearcamera {
    while [ -x $0 ]; do
        mjpg_streamer -i "input_opencv.so --filter /home/ubuntu/Documents/vision/build/libkryptoncv-back.so -d /dev/v4l/by-id/usb-HD_Camera_Manufacturer_HD_USB_Camera_SN0008-video-index0" -o "output_http.so -p 5802"
    done
}

frontcamera &
rearcamera &
