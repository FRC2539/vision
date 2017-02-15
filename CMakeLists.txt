cmake_minimum_required(VERSION 2.8)

project(vision)

find_package(OpenCV REQUIRED)
set(ntcore_LIBRARY "/usr/local/lib/libntcore.so")

include_directories(${OpenCV_INCLUDE_DIRS})

set(DEBUG FALSE CACHE BOOL "Display intermediate targets?")
set(DIRECTION "BOTH" CACHE STRING "Which direction the camera faces")

foreach(name IN ITEMS kryptoncv kryptoncv-back kryptoncv-front)
    add_library(${name} SHARED mjpgFilter.cpp)

    target_link_libraries(${name} ${OpenCV_LIBS})
    target_link_libraries(${name} ${ntcore_LIBRARY})
endforeach(name)

target_compile_definitions(kryptoncv PRIVATE DIRECTION_${DIRECTION})
target_compile_definitions(kryptoncv-back PRIVATE DIRECTION_BACK)
target_compile_definitions(kryptoncv-front PRIVATE DIRECTION_FRONT)

if (${DEBUG})
    message(STATUS "Compiling with DEBUG enabled")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
endif (${DEBUG})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

add_custom_target(front ALL COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/front_camera_data.xml ${PROJECT_BINARY_DIR}/front_camera_data.xml)
add_custom_target(back ALL COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/back_camera_data.xml ${PROJECT_BINARY_DIR}/back_camera_data.xml)
