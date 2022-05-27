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
#include <wiringPi.h>
#include <wiringSerial.h>
//#include "wiringPi/wiringPi/wiringPi.h"
//#include "wiringPi/wiringPi/wiringSerial.h"
#include "mavlink/common/mavlink.h"
#include "mavlink/ardupilotmega/ardupilotmega.h"
//#include "common/mavlink_msg_request_data_stream.h"
#include "serial_port.h"
#include <unistd.h>

// Mavlink Serial Setup
#define BAUDRATE 115200
#define UARTNAME "/dev/serial0"

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
    ARMED_STATE,
    TAKEOFF,
    STATIONARY,
    GUIDED_STATE,
    LANDING
} DRONE_STATE;

typedef enum {
    AUTO = 3,
    GUIDED_NOGPS = 20,
//    GUIDED = 4,
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
//    std::vector<int[]>

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
    bool drone_ready();

//  Serial Mavlink Methods
    int setup_serial();
    bool mavlink_receive_response();
    void mavlink_setup();
    void mavlink_heartbeat();
    void mavlink_request_data();
    int mavlink_receive_data();
    void mavlink_command_long(uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4,
                              float param5, float param6, float param7);
    bool mavlink_positioning_status();
    int mavlink_arm(int state = 0);
    int mavlink_disarm();
//    void mavlink_takeoff();

    int mavlink_set_flight_mode(FLIGHT_MODE mode);
//  Hardware Methods
    void toggle_pump();

    bool identify_table();


    // Get the current mode of the rover:
    int get_mode();
    int get_armed();
    int mavlink_handle_message(mavlink_message_t *msg);

    // Set the yaw and speed of the rover:
    int setAngleSpeed(float angle, float speed);


private:
    static std::string ini_sections(INIReader &reader);

    int r_baudrate;
    char *r_uart_name;
    Serial_Port serial_port;
    uint8_t r_armed;
    uint8_t r_mode;

    void verify_data();
};


#endif //DRONECLIENT_DRONE_H
