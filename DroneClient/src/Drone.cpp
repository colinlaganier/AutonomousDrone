/**
  **********************************************************************************
  * @file     Drone.cpp
  * @author   Colin Laganier
  * @version  V0.1
  * @date     2022-04-27
  * @brief   This file contains the constructor and methods for the drone.
  **********************************************************************************
  * @attention  Requires INIReader library to parse config file & MAVLink library for
  *             Ardupilot communication.
  */

// Include Files
#include "Drone.h"

/**************************************************************************************************************/
/*                                                CONSTRUCTORS                                                */
/**************************************************************************************************************/

Drone::Drone() {
//    sensor_status->fc = false;
//    sensor_status->imu = false;
//    sensor_status->uwb = false;
//    sensor_status->tcp = false;
}

Drone::~Drone() {
    delete position;
}

/**************************************************************************************************************/
/*                                                PUBLIC METHODS                                              */
/**************************************************************************************************************/

/**
  * @name   get_info
  * @brief  A method which parses data from config file and initializes public variables.
  * @param  file_name -> string for the file name.
  * @retval None.
  * @notes  None.
  */
bool Drone::get_info(std::string file_name)
{
//    Initialising a INI reader obj
    config_file = file_name;
    INIReader reader(file_name);

//    Check file loaded properly
    if (reader.ParseError() < 0) {
        std::cout << "Cannot load " << file_name;
        return false;
    }

    std::cout << "Config loaded from "<< file_name << ": found sections=" << ini_sections(reader) << "\n";

//    Assign values to variables from config file
    drone_id = reader.GetInteger("drone", "drone_id", 0);
    drone_port = reader.GetInteger("drone","drone_port", 0);
    drone_ip = reader.Get("drone", "drone_ip", "0");
    server_ip = reader.Get("server","server_ip","0");
    position = new int[3] {static_cast<int>(reader.GetInteger("drone", "longitude",0)),static_cast<int>(reader.GetInteger("drone", "latitude",0)),static_cast<int>(reader.GetInteger("drone", "altitude",0))};

//    Verify every value is inputted correctly
//    if (verify_data())
//        return true;
//    else
//        return false;

    return true;
}

std::string Drone::ini_sections(INIReader &reader)
{
    std::stringstream ss;
    std::set<std::string> sections = reader.Sections();
    for (std::set<std::string>::iterator it = sections.begin(); it != sections.end(); ++it)
        ss << *it << ",";
    return ss.str();
}

void Drone::toggle_sensor_imu() {
    sensor_status->imu = !sensor_status->imu;
}

void Drone::toggle_sensor_uwb() {
    sensor_status->uwb = !sensor_status->uwb;
}

void Drone::toggle_sensor_tcp() {
    sensor_status->tcp = !sensor_status->tcp;
}

void Drone::toggle_sensor_fc() {
    sensor_status->imu = !sensor_status->imu;
}

/**************************************************************************************************************/
/*                                               PRIVATE METHODS                                              */
/**************************************************************************************************************/

void Drone::verify_data() {
// TODO check every field filled
}

void Drone::get_status() {
    std::cout << "imu: " << ((sensor_status->imu) ? "on\n" : "off\n");
    std::cout << "uwb: " << ((sensor_status->uwb) ? "on\n" : "off\n");
    std::cout << "tcp: " << ((sensor_status->tcp) ? "on\n" : "off\n");
    std::cout << "fc: " << ((sensor_status->fc) ? "on\n" : "off\n");
}
