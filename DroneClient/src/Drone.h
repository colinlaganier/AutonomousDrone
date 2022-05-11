/**
  **********************************************************************************
  * @file     Drone.h
  * @author   Colin Laganier
  * @version  V0.1
  * @date     2022-04-27
  * @brief   This file contains the header information for the drone.
  **********************************************************************************
  * @attention  Requires INIReader library to parse config file & MAVLink library for
  *             Ardupilot communication.
  */

#ifndef DRONECLIENT_DRONE_H
#define DRONECLIENT_DRONE_H

#include <string>
#include <iostream>
#include "sstream"
#include "INIReader.h"

struct Sensors {
    bool uwb;
    bool imu;
    bool fc;
    bool tcp;
};

typedef enum {
    SETUP,
    DISARMED,
    STATIONARY,
    GUIDED,
    LANDING
} DRONE_STATE;

class Drone{
public:

//    Public Constructor
    Drone();
    virtual ~Drone();

//    Public Variables
    DRONE_STATE state = SETUP;
    std::string config_file;
    int drone_id;
    int drone_port;
    std::string drone_ip;
    std::string server_ip;
    int *position;        //position coordinates: x,y,z
    Sensors drone_sensor = { false, false, false, false };
    Sensors *sensor_status = &drone_sensor;

//  Public Methods
    bool get_info(std::string file_name);
    void get_status();
    void setup_uwb();
    void setup_imu();

    void toggle_sensor_imu();
    void toggle_sensor_uwb();
    void toggle_sensor_tcp();
    void toggle_sensor_fc();

private:
    static std::string ini_sections(INIReader &reader);
    void verify_data();

};


#endif //DRONECLIENT_DRONE_H
