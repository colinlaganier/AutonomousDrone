cmake_minimum_required(VERSION 3.16)
project(DroneClient)

set(CMAKE_CXX_STANDARD 14)

include_directories(${CMAKE_SOURCE_DIR}/inc)
link_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable(DroneClient src/main.cpp src/Drone.cpp src/Drone.h src/serial_port.cpp src/serial_port.h)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(DroneClient Threads::Threads wiringPi)
add_definitions(-lwiringPi)
