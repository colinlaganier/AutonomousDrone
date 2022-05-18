/**
  **********************************************************************************
  * @file     main.cpp
  * @author   Colin Laganier
  * @version  V0.1
  * @date     2022-04-15
  * @brief   This file contains the main control script of the drone.
  **********************************************************************************
  * @attention
  */

// Include Files
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <signal.h>
#include <chrono>
#include <unistd.h>
#include <thread>
#include "Drone.h"
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include "atomic"
//#include <wiringPi/wiringPi.h>

#define MESSAGE_BUFFER 128

mavlink_system_t mavlink_system = {
        1, // System ID
        1  // Component ID (MAV_COMPONENT value)
};


// Data shared between the two threads
struct Tcp_message{
    std::atomic_bool receive_flag;
    std::atomic_int send_flag;
    char send_message[MESSAGE_BUFFER];
    char receive_message[MESSAGE_BUFFER];
//    std::atomic_int send_message_len;
};

//  Function Prototypes
[[noreturn]] void control_loop(Drone *drone, Tcp_message *tcp_message);
[[noreturn]] void tcp_handler(int socket, char buffer[], Tcp_message *message);
void identify_message(Drone *drone, char message[], int message_head, char *message_response);
void send_tcp_message(Tcp_message *tcp_message, char *message);
void tcp_set_message(Tcp_message *message_object, char *new_message, bool send_or_receive);

//Tcp_message::Tcp_message(send_bool, receive_bool, send_char, send_int) {

//}

int main(){
    std::cout << "Drone Startup\n";
    Drone drone;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TCP Socket communication parameters
    int server_socket = 0;
    struct sockaddr_in server_address;
//    char* handshake_message;
    char buffer[MESSAGE_BUFFER];

    // Initialising variables
    Tcp_message tcp_message;
    tcp_message.send_flag = -1;
    tcp_message.receive_flag = false;
//    tcp_message.send_message_len = MESSAGE_BUFFER;
    char tcp_init_message[] = " ";
    tcp_set_message(&tcp_message, tcp_init_message, true);
    tcp_set_message(&tcp_message, tcp_init_message, false);

    if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cout << "\n Socket creation error \n";
        return -1;
    }

    server_address.sin_family = AF_INET;
    server_address.sin_port = htons((uint16_t)drone.drone_port);

    // Convert IPv4 and IPv6 addresses from text to binary
    if (inet_pton(AF_INET, drone.server_ip.c_str(), &server_address.sin_addr)<= 0) {
        std::cout << "\nInvalid address/ Address not supported \n";
        return -1;
    }

    if (connect(server_socket, (struct sockaddr*)&server_address,sizeof(server_address)) < 0) {
        std::cout << "\nConnection Failed \n";
        return -1;
    }

    // Verifies connection confirmation from server
    read(server_socket, buffer, MESSAGE_BUFFER);
    std::cout << "Connection confirmation:" << buffer << "\n";

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initializing UART communication

    if (drone.setup_serial() > 0) {
        std::cout << "Error initialising UART \n";
        return -1;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Create separate threads to receive base station commands while running
    std::thread network_thread(tcp_handler, server_socket, buffer, &tcp_message);
    std::thread control_thread(control_loop, &drone, &tcp_message);

    network_thread.join();
    control_thread.join();

    return 0;
}

void tcp_handler(int socket, char buffer[], Tcp_message *message){
    std::cout << "TCP Communication thread started\n";

//    int val_read;


    while(true)
    {
        if (message->send_flag){
            send(socket, message->send_message, MESSAGE_BUFFER, 0);
            message->send_flag = false;
        }
        else
//        val_read =
        read(socket, buffer, 1024);
        std::cout << "Read message:" << buffer << "\n";
        message->receive_flag = true;
        tcp_set_message(message, buffer, false);
   }
}

void send_tcp_message(Tcp_message *tcp_message, char *message){
    tcp_set_message(tcp_message, message, true);
}

FLIGHT_MODE identify_mode(char *mode_request){
    const char *mode_guided = "Guided";
    const char *mode_stabilize = "Stabilize";
    const char *mode_althold = "AltitudeHold";
    const char *mode_loiter = "Loiter";
    const char *mode_poshold = "PositionHold";
    const char *mode_rtl = "RTL";
    const char *mode_auto = "Auto";

    if (mode_request == mode_guided)
        return GUIDED_NOGPS;
    else if (mode_request == mode_stabilize)
        return STABILIZE;
    else if (mode_request == mode_althold)
        return ALT_HOLD;
    else if (mode_request == mode_loiter)
        return LOITER;
    else if (mode_request == mode_poshold)
        return POS_HOLD;
    else if (mode_request == mode_rtl)
        return RTL;
    else if (mode_request == mode_auto)
        return AUTO;
    return STABILIZE;
}


void identify_message(Drone *drone, char message[], int message_head, char *message_response){
    // Message format int:char[] -> ex: "1:Stabilize\0"
    char* message_tail;
    if (message[1] == ':'){
        message_tail = message + 2;
        std::cout << message_tail << '\n';
    }
    switch (message_head) {
        case -1: {
            std::cout << "idle mode";
            break;
        }
        case 0: {
            // Setup Mode
            if (drone->mavlink_positioning_status())
                drone->toggle_sensor_uwb();

            std::cout << "Check setup";
            break;
        }
        case 1: {
            // Set flight mode
            FLIGHT_MODE new_mode = identify_mode(message_tail);
            drone->mavlink_set_flight_mode(new_mode);
            break;
        }
        case 2: {
            // Guided coordinates
            break;
        }
        case 3:{
            drone->mavlink_arm();
            drone->state = ARMED;
            strcpy(message_response, "Drone Armed");
            // Arm Motors
//            if (drone->drone_sensor.uwb){
//                if (drone->flight_mode == STABILIZE || drone->flight_mode == AUTO) {
//                    drone->mavlink_arm();
//                    drone->state = ARMED;
//                    message_response = "Drone armed";
//                }
//                else {
//                    message_response = "Flight mode not set to stabilize";
//                }
//            }
//            message_response = "Positioning not ready";
            break;
        }
        case 4:{
            if (drone->state == ARMED && drone->flight_mode == AUTO){
//                drone->mavlink_takeoff();
                std::cout << "Takeoff\n";
//                drone->state = STATIONARY;
            }

            // Take off
            break;
        }
        case 5: {
            // Stabilize
            drone->mavlink_disarm();
            drone->state = DISARMED;
            strcpy(message_response, "Drone Disarmed");

            break;
        }
        case 6: {
            // Guided mode
            break;
        }
        case 7: {
            // Return home
            break;
        }
        case 8: {
            // Land
            break;
        }
         default:
            break;

    }
}

void tcp_set_message(Tcp_message *message_object, char *new_message, bool send_or_receive){
    char *destination = (send_or_receive) ? message_object->send_message :  message_object->receive_message;
    strncpy(destination, new_message, MESSAGE_BUFFER);
    *(destination+ MESSAGE_BUFFER) = 0;
//    message_object->send_message_len = strlen(new_message);
    std::cout << strlen(new_message) << '\n';
}

[[noreturn]] void control_loop(Drone *drone, Tcp_message *tcp_message){
    char message_response[256];

    while (true) {

        // Send heartbeat at a ~1Hz frequency
        unsigned long mavlink_current_heartbeat = millis();
        if (mavlink_current_heartbeat - drone->mavlink_previous_heartbeat >= drone->mavlink_interval_heartbeat) {
            drone->mavlink_previous_heartbeat = mavlink_current_heartbeat;
            drone->mavlink_heartbeat();
            drone->number_hbs++;

            // Periodic FC data request
            if (drone->number_hbs >= drone->setup_hbs){
                drone->mavlink_request_data();
                drone->number_hbs = 0;
            }
        }

        // Check if TCP command received from Ground Station
        if (tcp_message->receive_flag){
            int message_head = (int)tcp_message->receive_message[0];
            if (message_head != drone->get_state()){
                identify_message(drone, tcp_message->receive_message, message_head, message_response);
                tcp_set_message(tcp_message, message_response, true);
            }
        }

//        if (drone->state == TAKEOFF)
//            std::cout <<

        //  Receiving MAVLINK data
        drone->mavlink_receive_data();

        // Toggle spray if over a table
//        if (drone->spray_state != drone->identify_table())
//            drone->toggle_pump();

    }
}