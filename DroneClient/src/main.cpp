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
#include <cstdlib>
#include <signal.h>
#include "config.h"
#include "Drone.h"
#include "Socket.h"

int main(void)
{

    Socket *server_socket = new Socket(AF_INET, SOCK_STREAM,);
    return 0;
}

bool parseINI(){
    return true;
}