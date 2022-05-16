cmake_minimum_required(VERSION 3.22)
project(DroneServer)

set(CMAKE_CXX_STANDARD 14)

add_executable(DroneServer src/main.cpp src/Socket.cpp src/Socket.h src/TCP_test.cpp)
