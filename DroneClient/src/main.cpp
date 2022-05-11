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
#include <wiringPi/wiringPi.h>
#include <wiringPi/wiringSerial.h>
#include "mavlink/ardupilotmega/mavlink.h"
#include "Drone.h"
//#include "Socket.h"
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

#define PORT 8888

//  Function Prototypes
bool tpc_verify(Drone &drone, Socket *server_socket);
void tcp_read_test(Socket *server_socket, int *message);
//void control_loop(Drone *drone, int *serial_comm, int *message);
void tcp_read(int socket, char buffer[]);
void test_thread();

int main()
{
    std::cout << "Drone Startup\n";
    Drone drone;

    std::string config_file = "../src/drone.ini";
    if (!drone.get_info(config_file))
        return 0;

//    Initializing socket for communication
//    int tcp_message = -1;
//    auto *server_socket = new Socket(AF_INET, SOCK_STREAM,0);
//    cout << "Socket Connecting...\n";
//    server_socket->connect(drone.server_ip, drone.drone_port);
//    cout << "Socket Connected\n";
//    server_socket->socket_write("Drone " + std::to_string(drone.drone_id) + " init socket");

//    if (!tpc_verify(drone,server_socket)) {
//        return 0;
//    }

    int server_socket = 0, valread;
    struct sockaddr_in serv_addr;
    char* hello = "Hello from client";
    char buffer[1024] = { 0 };
    if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons((uint16_t)drone.drone_port);

    // Convert IPv4 and IPv6 addresses from text to binary
    if (inet_pton(AF_INET, drone.server_ip.c_str(), &serv_addr.sin_addr)<= 0) {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    if (connect(server_socket, (struct sockaddr*)&serv_addr,sizeof(serv_addr)) < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }

//    send(server_socket, hello, strlen(hello), 0);

    valread = read(server_socket, buffer, 1024);
    std::cout << "Read message:" << buffer << "\n";

//    tcp_read(server_socket, buffer);

//    Initializing UART communication
    int fd ;
    int count ;
    unsigned int nextTime ;

//    TODO check if 115200 or 57600
//    if ((fd = serialOpen("/dev/ttyAMA0", 115200)) < 0)
//    {
//        cout << "Unable to open serial device\n";
//        return 1 ;
//    }
//    if (wiringPiSetup() == -1)
//    {
//        cout << "Unable to start wiringPi\n";
//        return 1 ;
//    }

//    Initializing MAVLINK communication


// Create separate threads to receive base station commands while running
    std::thread network_thread(tcp_read,server_socket, buffer);
//    thread network_thread(tcp_read,server_socket,&tcp_message);
//    thread control_thread(control_loop, &drone, &fd,&tcp_message);

    network_thread.join();
//    control_thread.join();

    return 0;
}
//  Verifies the MAVLINK connection is established
bool tpc_verify(Drone &drone, Socket *server_socket){
    cout << "TCP Connection Verification";
    int seconds = 10;// Allowed response delay
    vector<Socket> reads(1);
    reads[0] = *server_socket;
    int connection_counter = 0;
    int connection_attempts = 3;

//    Verify connection has been established
//    while (!drone.sensor_status->tcp)
//    {
    if(Socket::select(&reads, nullptr, nullptr, seconds) < 1){//Socket::select waits until server_socket receives some input (for example the answer from google.com)
        connection_counter++;
        std::cout << "TCP Socket connection error " << connection_counter << "\n" ;
    }
    else{
        string buffer = "Connection established I think";
        server_socket->socket_write(buffer);
        server_socket->socket_read(buffer, 1024);//Read 1024 bytes of the answer
        cout << buffer << endl;
        drone.toggle_sensor_tcp();
    }

//        if (connection_counter > connection_attempts)
//            return false;
//    }
    return true;
}
void tcp_read(int socket, char buffer[]){
    std::cout << "In TCP Read\n";
    int valread;
    char* hello = "Hello from client";

    while(TRUE)
    {
//        send(socket,)
        send(socket, hello, strlen(hello), 0);
        cout << "Hello message sent\n";
        valread = read(socket, buffer, 1024);
        cout << "Read message:" << buffer << "\n";
    }
}

void tcp_read_test(Socket *server_socket, int *message){
    cout << "TCP read start";
    vector<Socket> reads(1);
    reads[0] = *server_socket;
    int seconds = 10; //Wait 10 seconds for input
    while (true) {
        if(Socket::select(&reads, NULL, NULL, seconds) < 1){ //Socket::select waits until masterSocket reveives some input (for example a message)
            //No Input
            cout << "No input\n";
            *message = -1;
        }else{
//            *flag = true;
            string buffer = "Connection established I think round 2";
            server_socket->socket_write(buffer);
            server_socket->socket_read(buffer, 1024); //Read 1024 bytes of the stream
            cout << buffer;
//            *message = stoi(buffer);

//            server_socket->socket_write(buffer); //Sends the input back to the client (echo)
        }
    }
}


//void control_loop(Drone *drone, int *serial_comm, int *message){
//    while (true) {
//
//        if (*message != -1 && *message != drone->state){
//            switch (*message) {
//                case 0:
//
//                    break;
//                case 1:
//                    break;
//                case 2:
//                    break;
//            }
//        }
//
//        while (serialDataAvail(*serial_comm))
//        {
//            cout << serialGetchar (*serial_comm);
//        }
////        serialPutchar (fd, count) ;
//
//    }
//}