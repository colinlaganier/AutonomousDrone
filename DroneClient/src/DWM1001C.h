/**
  **********************************************************************************
  * @file     DWM1001C.h
  * @author   Colin Laganier
  * @version  V0.1
  * @date     2022-05-01
  * @brief   This file contains the header information for the DWM1001C positioning.
  **********************************************************************************
  * @attention  Requires DWM1001 API library & WiringPi for hardware communication.
  */


#ifndef DRONECLIENT_DWM1001C_H
#define DRONECLIENT_DWM1001C_H

#include "dwm1001/dwm_api.h"
#include <wiringPi/wiringPi.h>
#include <wiringPi/wiringSerial.h>
#include <wiringPi/wiringPiSPI.h>

class DWM1001C {
public:

    //    Public Constructor
    DWM1001C();
    virtual ~DWM1001C();

    //    Public Variables

};


#endif //DRONECLIENT_DWM1001C_H
