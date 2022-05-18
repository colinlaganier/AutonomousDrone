/**
  **********************************************************************************
  * @file     Drone.h
  * @author   Colin Laganier
  * @version  V0.1
  * @date     2022-04-27
  * @brief   This file contains the header information for the drone.
  **********************************************************************************
  * @attention  Requires INIReader library to parse config file, MAVLink library for
  *             Ardupilot communication & WiringPi for hardware communication.
  */

#ifndef DRONECLIENT_DRONE_H
#define DRONECLIENT_DRONE_H

#include <string>
#include <iostream>
#include "sstream"
#include "INIReader.h"
#include <wiringPi/wiringPi.h>
#include <wiringPi/wiringSerial.h>
#include "mavlink/ardupilotmega/mavlink.h"
//#include "common/mavlink_msg_request_data_stream.h"


struct Sensors {
    bool uwb;
    bool imu;
    bool fc;
    bool tcp;
};

struct Waypoint {
    uint32_t latitude;
    uint32_t longitude;
    float altitude;

};

typedef enum {
    SETUP,
    DISARMED,
    ARMED,
    TAKEOFF,
    STATIONARY,
    GUIDED,
    LANDING
} DRONE_STATE;

typedef enum {
    AUTO = 3,
    GUIDED_NOGPS = 20,
    STABILIZE = 0,
    ALT_HOLD = 2,
    LOITER = 5,
    POS_HOLD = 16,
    RTL = 6
} FLIGHT_MODE;

class Drone{
public:

//    Public Constructor
    Drone();
    virtual ~Drone();

//    Public Variables
    DRONE_STATE state = SETUP;
    FLIGHT_MODE flight_mode = STABILIZE;
    std::string config_file = "../src/drone.ini";
    int drone_id;
    int drone_port;
    std::string drone_ip;
    std::string server_ip;
    int *position;        //position coordinates: x,y,z
    int pump_pwm;
    bool spray_state = false;
    Sensors drone_sensor = { false, false, false, false };
    Sensors *sensor_status = &drone_sensor;
    // guided message -> [x,y,z]
    std::vector<int[]>

//  MAVLINK Variables
    int serial;
    unsigned long mavlink_previous_heartbeat = 0;     // will store las
    unsigned long mavlink_interval_heartbeat = 1000;  // next interval to count
    const int setup_hbs = 60;                      // number of heartbeats to wait before activating STREAMS
    int number_hbs = setup_hbs;
    int mavlink_sys_id;
    int mavlink_comp_id;
    int mavlink_target_sys_id;
    int mavlink_target_comp_id;
    int mavlink_type;

//  Public Methods
//  Drone Data Methods
    void get_info(std::string file_name);
    void get_status();
    DRONE_STATE get_state();

//  Sensor Methods
    void setup_uwb();
    void setup_imu();
    void toggle_sensor_imu();
    void toggle_sensor_uwb();
    void toggle_sensor_tcp();
    void toggle_sensor_fc();

//  Serial Mavlink Methods
    int setup_serial(int *serial);
    void mavlink_setup();
    void mavlink_heartbeat();
    void mavlink_request_data();
    void mavlink_receive_data();
    void mavlink_command_long(uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4,
                              float param5, float param6, float param7);
    bool mavlink_positioning_status();
    void mavlink_arm();
    void mavlink_disarm();
//    void mavlink_takeoff();

    void mavlink_set_flight_mode(FLIGHT_MODE mode);
//  Hardware Methods
    void toggle_pump();

    bool identify_table();
private:
    static std::string ini_sections(INIReader &reader);

    void verify_data();
};


#endif //DRONECLIENT_DRONE_H
