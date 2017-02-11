cmake_minimum_required(VERSION 2.8)

project(vision)

find_package(OpenCV REQUIRED)
set(ntcore_LIBRARY "/usr/local/lib/libntcore.so")

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(kryptoncv SHARED mjpgFilter.cpp)

target_link_libraries(kryptoncv ${OpenCV_LIBS})
target_link_libraries(kryptoncv ${ntcore_LIBRARY})

set(DEBUG FALSE CACHE BOOL "Display intermediate targets?")
set(DIRECTION "FRONT" CACHE STRING "Which direction the camera faces")

if (${DEBUG})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
endif

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

