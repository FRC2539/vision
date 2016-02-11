cmake_minimum_required(VERSION 2.8)

project(vision)

find_package(OpenCV REQUIRED)
set(ntcore_LIBRARY "/usr/local/lib/libntcore.so")

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(kryptoncv SHARED mjpgFilter.cpp)

target_link_libraries(kryptoncv ${OpenCV_LIBS})
target_link_libraries(kryptoncv ${ntcore_LIBRARY})

target_compile_definitions(kryptoncv PRIVATE DIRECTION_${DIRECTION})

add_library(kryptoncv-back SHARED mjpgFilter.cpp)
add_library(kryptoncv-front SHARED mjpgFilter.cpp)

target_link_libraries(kryptoncv-back ${OpenCV_LIBS})
target_link_libraries(kryptoncv-back ${ntcore_LIBRARY})
target_link_libraries(kryptoncv-front ${OpenCV_LIBS})
target_link_libraries(kryptoncv-front ${ntcore_LIBRARY})

target_compile_definitions(kryptoncv-back PRIVATE DIRECTION_BACK)
target_compile_definitions(kryptoncv-front PRIVATE DIRECTION_FRONT)

set(DEBUG FALSE CACHE BOOL "Display intermediate targets?")
set(DIRECTION "FRONT" CACHE STRING "Which direction the camera faces")

if (${DEBUG})
    message(STATUS "Compiling with DEBUG enabled")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
endif (${DEBUG})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

