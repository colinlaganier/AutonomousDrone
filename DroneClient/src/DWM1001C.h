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

#define INTERFACE_NUMBER 1;
#define NUM_ANCHORS 4
#define MSG_HEADER          0x01
#define MSGID_BEACON_CONFIG 0x02
#define MSGID_BEACON_DIST   0x03
#define MSGID_POSITION      0x04

#include <iostream>
#include "hal/hal.h"
#include "dwm1001/dwm_api.h"
#include <wiringPi/wiringPi.h>
#include <wiringPi/wiringSerial.h>

class DWM1001C {
public:

    //    Public Constructor
    DWM1001C();
    virtual ~DWM1001C();

    //    Public Variables
    const int wait_period = 1000;
    unsigned long uwb_last_sensing;
    int uwb_serial;
    dwm_cfg_tag_t cfg_tag;
    dwm_cfg_t cfg_node;
    dwm_loc_data_t loc;
    dwm_pos_t pos;
    uint16_t anchor_id[4] = { 0x601C, // (0,0)
                              0x6020, // x-axis
                              0x6057, // y-axis
                              0x605E};
    int32_t anchors_x[NUM_ANCHORS] = {0,     10000, 0,     10000};    // anchor x-coorindates in mm (horizontal)
    int32_t anchors_y[NUM_ANCHORS] = {0,     0,     10000, 10000};    // anchor y-coordinates in mm (vertical)
    int32_t heights[NUM_ANCHORS] =   {-1200, -1200, -1200, -1200};    // anchor z-coordinates in mm (1.2m above vehicle's starting altitude)
    union vehicle_position_msg {
        struct {
            int32_t x;
            int32_t y;
            int32_t z;
            int16_t position_error;
        } info;
        uint8_t buf[14];
    };

    // Public Methods
    void dwm_setup();
    void dwm_set_params(dwm_cfg_tag_t *cfg_tag);
    bool dwm_verify_config();
    void dwm_get_position();
    void send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[]);
    int setup_serial();
};


#endif //DRONECLIENT_DWM1001C_H
