#!/bin/bash

function camera {
    while [ -x $0 ]; do
        python3 -OO /home/ubuntu/Documents/vision/robovision.py
    done
}



camera &
