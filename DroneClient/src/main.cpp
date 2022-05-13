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
#include <unistd.h>
#include <thread>

#include "Drone.h"
 #include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include "atomic"

#define PORT 8888

//  Function Prototypes
bool tpc_verify(Drone &drone, Socket *server_socket);
void tcp_read_test(Socket *server_socket, int *message);
//void control_loop(Drone *drone, int *serial_comm, int *message);
void tcp_handler(int socket, char buffer[],int *message, int *sub_message, bool *send_flag, char send_message[]);
void test_thread();

// Data shared between the two threads
struct Tcp_message{
    std::atomic_int send_flag;
    std::atomic_bool receive_flag;
    std::atomic_char *send_message;
    std::atomic_int send_message_len;
};

//Tcp_message::Tcp_message(send_bool, receive_bool, send_char, send_int) {

//}

int main(){
    std::cout << "Drone Startup\n";
    Drone drone;

    // Loads drone parameters
    std::string config_file = "../src/drone.ini";
    if (!drone.get_info(config_file))
        return -1;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TCP Socket communication parameters
//    int tcp_message = -1;
    int server_socket = 0, val_read;
    struct sockaddr_in serv_addr;
    char* handshake_message;
    snprintf(handshake_message, 24, "Connection from drone %d", drone.drone_id);
    char buffer[1024] = { 0 };

    Tcp_message tcp_message;
//    = {-1, false, nullptr, 0};
    tcp_message.send_flag = -1;
    tcp_message.receive_flag = false;
    tcp_message.send_message_len = 0;
    char *test = "test";
    strcpy(tcp_message.send_message, test);
//    TODO fix char pointer issue
//  atomic char cannot be an array and not sure how to modify string otherwise

    // TCP Socket setup
    if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cout << "\n Socket creation error \n";
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons((uint16_t)drone.drone_port);

    // Convert IPv4 and IPv6 addresses from text to binary
    if (inet_pton(AF_INET, drone.server_ip.c_str(), &serv_addr.sin_addr)<= 0) {
        std::cout << "\nInvalid address/ Address not supported \n";
        return -1;
    }

    if (connect(server_socket, (struct sockaddr*)&serv_addr,sizeof(serv_addr)) < 0) {
        std::cout << "\nConnection Failed \n";
        return -1;
    }

    // Verifies connection confirmation from server
    val_read = read(server_socket, buffer, 1024);
    std::cout << "Connection confirmation:" << buffer << "\n";

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initializing UART communication

    if (drone.setup_serial(&drone.serial) > 0) {
        std::cout << "Error initialising UART \n";
        return -1;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Create separate threads to receive base station commands while running
    std::thread network_thread(tcp_handler,server_socket, buffer);
    std::thread control_thread(control_loop, &drone, &fd,&tcp_message);

    network_thread.join();
//    control_thread.join();

    return 0;
}

//void tcp_handler(int socket, char buffer[], int *message, int *sub_message, bool *send_flag, char send_message[]){
void tcp_handler(int socket, char buffer[], Tcp_message message){
    std::cout << "TCP Communication thread started\n";

    int val_read;


    while(true)
    {
        if (message.send_flag){
            send(socket, message.send_message, message.send_message_len, 0);
        }
        else
//        if
//        send(socket, hello, strlen(hello), 0);
//        cout << "Hello message sent\n";
        val_read = read(socket, buffer, 1024);
        std::cout << "Read message:" << buffer << "\n";
        
    }
}



int identify_message(int message, int sub_message = 0){
    switch (message) {
        case -1:
            break;
        case 0:
            // Idle Mode
            break;
        case 1:
            // Send status/sensor data
            break;
        case 2:
            // Guided coordinates
            break;
        case 3:
            // Arm Motors
            break;
        case 4:
            // Take off
            break;
        case 5:
            // Stabilize
            break;
        case 6:
            // Guided mode
            break;
        case 7:
            // Return home
            break;
        case 8:
            // Land
            break;
    }
}


[[noreturn]] void control_loop(Drone *drone, int *serial_comm, int *message){
    while (true) {

        drone->mavlink_heartbeat();


        if (*message != -1 && *message != drone->state){
            switch (*message) {
                case 0:

                    break;
                case 1:
                    break;
                case 2:
                    break;
            }
        }

        while (serialDataAvail(*serial_comm))
        {
            std::cout << serialGetchar (*serial_comm);
        }
//        serialPutchar (fd, count) ;

    }
}