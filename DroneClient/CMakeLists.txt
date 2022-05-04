cmake_minimum_required(VERSION 3.16)
project(DroneClient)

set(CMAKE_CXX_STANDARD 14)

include_directories(${CMAKE_SOURCE_DIR}/inc)
link_directories(${CMAKE_SOURCE_DIR}/lib)
add_executable(DroneClient src/main.cpp src/UART.h src/UWB.cpp src/TCP_Socket.cpp src/Mavlink.cpp src/Drone.cpp src/Drone.h src/config.h src/Socket.cpp src/Socket.h src/dwm_api.cpp src/dwm_api.h)