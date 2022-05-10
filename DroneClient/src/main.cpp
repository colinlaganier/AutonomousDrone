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
#include "Socket.h"

//  Function Prototypes
bool tpc_verify(Drone &drone, Socket *server_socket);
void tcp_read(Socket *server_socket, int *message);
void control_loop(Drone *drone, int *serial_comm, int *message);

int main()
{
    Drone drone;

    std::string config_file = "../src/drone.ini";
    if (!drone.get_info(config_file))
        return 0;

//    Initializing socket for communication
    int tcp_message = -1;
    Socket *server_socket = new Socket(AF_INET, SOCK_STREAM,0);
    server_socket->connect(drone.server_ip, drone.drone_port);
    server_socket->socket_write("Drone " + std::to_string(drone.drone_id) + " init socket\n");

    if (!tpc_verify(drone,server_socket)) {
        return 0;
    }

//    Initializing UART communication
    int fd ;
    int count ;
    unsigned int nextTime ;

//    TODO check if 115200 or 57600
    if ((fd = serialOpen("/dev/ttyAMA0", 115200)) < 0)
    {
        cout << "Unable to open serial device\n";
        return 1 ;
    }
    if (wiringPiSetup() == -1)
    {
        cout << "Unable to start wiringPi\n";
        return 1 ;
    }

//    Initializing MAVLINK communication


// Create separate threads to receive base station commands while running
    thread network_thread(tcp_read,server_socket,&tcp_message);
    thread control_thread(control_loop, &drone, &fd,&tcp_message);

    network_thread.join();
    control_thread.join();

    return 0;
}
//  Verifies the MAVLINK connection is established
bool tpc_verify(Drone &drone, Socket *server_socket){
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

void tcp_read(Socket *server_socket, int *message){
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
            string buffer;
            server_socket->socket_read(buffer, 1024); //Read 1024 bytes of the stream
            cout << buffer;
            *message = stoi(buffer);

//            server_socket->socket_write(buffer); //Sends the input back to the client (echo)
        }
    }
}

void control_loop(Drone *drone, int *serial_comm, int *message){
    while (true) {

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
            cout << serialGetchar (*serial_comm);
        }
//        serialPutchar (fd, count) ;

    }
}