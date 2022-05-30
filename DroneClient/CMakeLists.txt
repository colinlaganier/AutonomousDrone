cmake_minimum_required(VERSION 3.16)
project(DroneClient)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

include_directories("${CMAKE_SOURCE_DIR}/inc")
link_directories(${CMAKE_SOURCE_DIR}/lib)

#add_library(dwm1001 src/dwm1001/dwm_api.c src/dwm1001/dwm_api.h src/dwm1001/lmh.c
#        src/dwm1001/lmh.h
#        src/dwm1001/lmh_spirx.c
#        src/dwm1001/lmh_spirx.h
#        src/dwm1001/dwm1001_tlv.h)
#
#add_library(hal src/hal/hal.c
#        src/hal/hal.h
#        src/hal/hal_gpio.c
#        src/hal/hal_gpio.h
#        src/hal/hal_interface.h
#        src/hal/hal_log.c
#        src/hal/hal_log.h
#        src/hal/hal_spi.c
#        src/hal/hal_spi.h
#        src/hal/hal_uart.c
#        src/hal/hal_uart.h)

set_source_files_properties(filename.c PROPERTIES LANGUAGE CXX )

add_definitions(-INTERFACE_NUMBER=1)
add_executable(DroneClient src/main.cpp src/Drone.cpp src/Drone.h src/Serial.cpp src/Serial.h src/DWM1001C.cpp src/DWM1001C.h
        src/dwm_api.c src/dwm_api.h src/lmh.c
        src/lmh.h
        src/lmh_spirx.c
        src/lmh_spirx.h
        src/dwm1001_tlv.h
        src/hal.c
        src/hal.h
        src/hal_gpio.c
        src/hal_gpio.h
        src/hal_interface.h
        src/hal_log.c
        src/hal_log.h
        src/hal_spi.c
        src/hal_spi.h
        src/hal_uart.c
        src/hal_uart.h)

#add_library(dwm1001 STATIC)
#target_sources(dwm1001
#        PRIVATE
#        src/dwm1001/dwm_api.c
#        src/dwm1001/dwm_api.h
#        src/dwm1001/lmh.c
#        src/dwm1001/lmh.h
#        src/dwm1001/lmh_spirx.c
#        src/dwm1001/lmh_spirx.h
#        src/dwm1001/dwm1001_tlv.h
#        )
#
#add_library(hal STATIC)
#target_sources(hal
#        PRIVATE
#        src/hal/hal.c
#        src/hal/hal.h
#        src/hal/hal_gpio.c
#        src/hal/hal_gpio.h
#        src/hal/hal_interface.h
#        src/hal/hal_log.c
#        src/hal/hal_log.h
#        src/hal/hal_spi.c
#        src/hal/hal_spi.h
#        src/hal/hal_uart.c
#        src/hal/hal_uart.h)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(DroneClient Threads::Threads wiringPi)
#hal dwm1001)
add_definitions(-lwiringPi)

