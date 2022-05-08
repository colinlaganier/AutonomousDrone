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
#include "Drone.h"
#include "Socket.h"

bool verify_connection(Drone &drone, Socket *server_socket){
    int seconds = 10;// Allowed response delay
    vector<Socket> reads(1);
    reads[0] = *server_socket;
    int connection_counter = 0;
    int connection_attempts = 3;

//    Verify connection has been established
    while (!drone.sensor_status->tcp)
    {
        if(server_socket->select(&reads, NULL, NULL, seconds) < 1){//Socket::select waits until sock receives some input (for example the answer from google.com)
            connection_counter++;
            std::cout << "TCP Socket connection error " << connection_counter << "\n" ;
        }
        else{
            std::string buffer;
            server_socket->socket_read(buffer, 1024);//Read 1024 bytes of the answer
            cout << buffer << endl;
            drone.toggle_sensor_tcp();
        }

        if (connection_counter > connection_attempts)
            return false;
    }
    return true;
}

int main()
{
    Drone drone;

    std::string config_file = "../src/drone.ini";
    if (!drone.get_info(config_file))
        return 0;

//    Initializing socket for communication
    Socket *server_socket = new Socket(AF_INET, SOCK_STREAM,0);
    server_socket->connect(drone.server_ip, drone.drone_port);
    server_socket->socket_write("Drone " + std::to_string(drone.drone_id) + " init socket\n");

    if (!verify_connection(drone,server_socket)) {
        return 0;
    }

    while (true)
    {
        vector<Socket> reads(1);
        reads[0] = *server_socket;
        int seconds = 10; //Wait 10 seconds for input
        if(Socket::select(&reads, NULL, NULL, seconds) < 1){ //Socket::select waits until masterSocket reveives some input (for example a message)
            //No Input
            continue;
        }else{
            string buffer;
            server_socket->socket_read(buffer, 1024); //Read 1024 bytes of the stream

            server_socket->socket_write(buffer); //Sends the input back to the client (echo)
        }
    }


    return 0;
}