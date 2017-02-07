#!/bin/bash
filename=$(basename "$1")
g++ --std=c++14 -pedantic -ansi `pkg-config --cflags --libs opencv` -o ${filename%.*} $1

