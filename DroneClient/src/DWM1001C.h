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

#define INTERFACE_NUMBER 1
#define TARGET 0
#define PRINT_LVL 3
#define NUM_ANCHORS 4
#define MSG_HEADER          0x01
#define MSGID_BEACON_CONFIG 0x02
#define MSGID_BEACON_DIST   0x03
#define MSGID_POSITION      0x04
#define UWB_BAUDRATE 115200
#define UWB_UARTNAME "/dev/ttyUSB0"

#include <iostream>
#include "hal.h"
#include "dwm_api.h"
#include "wiringPi/wiringPi/wiringPi.h"
#include "wiringPi/wiringPi/wiringSerial.h"
//#include <wiringPi/wiringPi.h>
//#include <wiringPi/wiringSerial.h>

// Ardupilot message structures
union beacon_config_msg {
    struct {
        uint8_t beacon_id;
        uint8_t beacon_count;
        int32_t x;
        int32_t y;
        int32_t z;
    } info;
    uint8_t buf[14];
};
union beacon_distance_msg {
    struct {
        uint8_t beacon_id;
        uint32_t distance;
    } info;
    uint8_t buf[5];
};
union vehicle_position_msg {
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
        int16_t position_error;
    } info;
    uint8_t buf[14];
};

typedef struct __attribute__((packed))_coordinates {
    int32_t x;
    int32_t y;
    int32_t z;
    int16_t position_error;
}coordinates_t;

typedef struct __attribute__((packed))_device_range {
    uint32_t timestamp;
    uint32_t distance;
}device_range_t;


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

//    unsigned long long anchor_id[4] = {0xd837be81e838, // (0,0)
//                                       0x530cbe81e838, // x-axis
//                                       0x5d86be81e838, // y-axis
//                                       0xc604be81e838};
    // ID anchor modified to fit pozyx format
    uint16_t anchor_id[4] = { 0x6001, // (0,0)
                              0x6002, // x-axis
                              0x6003, // y-axis
                              0x6004};
    int32_t anchors_x[NUM_ANCHORS] = {0, 2500, 0, 2500};    // anchor x-coorindates in mm (horizontal)
    int32_t anchors_y[NUM_ANCHORS] = {0, 0, 2500, 2500};    // anchor y-coordinates in mm (vertical)
    int32_t anchors_z[NUM_ANCHORS] = {-1200, -1200, -1200,-1200};    // anchor z-coordinates in mm (1.2m above vehicle's starting altitude)

    // Public Methods
    void dwm_setup();
    int dwm_setup_serial();
    void dwm_set_params(dwm_cfg_tag_t *cfg_tag);
    bool dwm_verify_config();
    void dwm_send_beacon_config();
    void dwm_get_position();
    void dwm_get_ranges();
    void dwm_send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[]);
    void dwm_send_beacon_distance(uint8_t beacon_id, uint32_t distance_mm);
    void dwm_send_vehicle_position(coordinates_t& position);
};

#endif //DRONECLIENT_DWM1001C_H
