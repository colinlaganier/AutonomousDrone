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

//Drone::Drone() {
////    spray_state = false;
//    get_info(config_file);
//    mavlink_previous_heartbeat = millis();
//    std::cout << "Heartbeat millis " << mavlink_previous_heartbeat << "\n";
//    std::cout << "Constructor call finished\n";
//}

Drone::Drone():serial_port((char*)UARTNAME, BAUDRATE)
{
    //initialization of the serial communication:
    std::cout << "Serial - interface start\n";
    serial_port.start();
    toggle_sensor_fc();

    get_info(config_file);
    mavlink_previous_heartbeat = millis();
    std::cout << "Constructor call finished\n";

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
void Drone::get_info(std::string file_name)
{
//    Initialising a INI reader obj
    config_file = file_name;
    INIReader reader(file_name);

//    Check file loaded properly
    if (reader.ParseError() < 0) {
        std::cout << "Cannot load " << file_name;
    }
    else {
        std::cout << "config loaded from "<< file_name << ": found sections=" << ini_sections(reader) << "\n";

        //    Assign values to variables from config file
        drone_id = reader.GetInteger("drone", "drone_id", 0);
        drone_port = reader.GetInteger("drone","drone_port", 0);
        drone_ip = reader.Get("drone", "drone_ip", "0");
        server_ip = reader.Get("server","server_ip","0");
        position = new int[3] {static_cast<int>(reader.GetInteger("drone", "longitude",0)),static_cast<int>(reader.GetInteger("drone", "latitude",0)),static_cast<int>(reader.GetInteger("drone", "altitude",0))};
        mavlink_sys_id = reader.GetInteger("mavlink","system_id", 0);
        mavlink_comp_id = reader.GetInteger("mavlink","component_id", 0);
        mavlink_type = MAV_AUTOPILOT_INVALID;

        //    Verify every value is inputted correctly
        //    if (verify_data())
        //        return true;
        //    else
        //        return false;

    }
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
    sensor_status->fc = !sensor_status->fc;
}

bool Drone::drone_ready(){
    if (drone_sensor.fc == false)
        return false;
    if (drone_sensor.tcp == false)
        return false;
    //if (sensor_status.uwb == false)
        //return false;
    //if (sensor_status.imu == false)
        //return false;
}

int Drone::setup_serial() {
    // Initializing UART communication
    serial = serialOpen("/dev/ttyUSB0", 115200);
    if (serial < 0)
    {
        std::cout << "Unable to open serial device\n";
        return 1 ;
    }
    if (wiringPiSetup() == -1)
    {
        std::cout << "Unable to start wiringPi\n";
        return 1 ;
    }
    char init_message[] = {"This is the init message"};
    serialPuts(serial,init_message);
    return 0;
}

//void Drone::mavlink_setup() {
//    std::cout << "Mavlink Setup\n";
//    int mavlink_attempt = 0;
//    int mavlink_setup_interval = 500;
//
//    unsigned long mavlink_setup_attempt = millis();
//    unsigned long mavlink_attempt_current = mavlink_setup_attempt;
//
//    bool mavlink_response = false;
//
//    while (mavlink_attempt < 20 ){
//        mavlink_heartbeat();
//
//        bool response = mavlink_receive_response();
//
//        if (response){
//            mavlink_response = true;
//            break;
//        }
//        else
//            mavlink_attempt++;
//
//
//        while ((mavlink_attempt_current - mavlink_setup_attempt) < mavlink_setup_interval)
//            mavlink_attempt_current = millis();
//    }
//
//    if (mavlink_response)
//        toggle_sensor_fc();
//}


void Drone::mavlink_heartbeat() {
//    int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
//    int compid = 158;                ///< The component sending the message
    int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

//    uint8_t system_type = MAV_TYPE_GENERIC;
    uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

    uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
    uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t mavlink_buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

    // Copy the message to the buffer
    uint16_t len = mavlink_msg_to_send_buffer(mavlink_buffer, &msg);
    char mavlink_serial[len+1];
    memcpy(mavlink_buffer, mavlink_serial, len+1);
    mavlink_serial[-1] = '\0';
    serialPuts(serial, mavlink_serial);
//    std::cout << "Mavlink Serial: " << mavlink_serial << '\n';
}

void Drone::mavlink_request_data() {
    mavlink_message_t msg;
    uint8_t mavlink_buffer[MAVLINK_MAX_PACKET_LEN];

    /*
     * Definitions are in common.h: enum MAV_DATA_STREAM
     *
     * MAV_DATA_STREAM_ALL=0, // Enable all data streams
     * MAV_DATA_STREAM_RAW_SENSORS=1,  Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
     * MAV_DATA_STREAM_EXTENDED_STATUS=2,  Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
     * MAV_DATA_STREAM_RC_CHANNELS=3,  Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
     * MAV_DATA_STREAM_RAW_CONTROLLER=4,  Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
     * MAV_DATA_STREAM_POSITION=6,  Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
     * MAV_DATA_STREAM_EXTRA1=10,  Dependent on the autopilot
     * MAV_DATA_STREAM_EXTRA2=11,  Dependent on the autopilot
     * MAV_DATA_STREAM_EXTRA3=12,  Dependent on the autopilot
     * MAV_DATA_STREAM_ENUM_END=13,
     */

    // To be setup according to the needed information to be requested from the Pixhawk
    const int maxStreams = 2;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
    const uint16_t MAVRates[maxStreams] = {0x02, 0x05};


    for (int i = 0; i < maxStreams; i++) {
        /*
         * mavlink_msg_request_data_stream_pack(system_id, component_id,
         *    &msg,
         *    target_system, target_component,
         *    MAV_DATA_STREAM_POSITION, 10000000, 1);
         *
         * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id,
         *    mavlink_message_t* msg,
         *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id,
         *    uint16_t req_message_rate, uint8_t start_stop)
         *
         */
        mavlink_msg_request_data_stream_pack(10, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
        uint16_t len = mavlink_msg_to_send_buffer(mavlink_buffer, &msg);
        char mavlink_serial[len+1];
        memcpy(mavlink_buffer, mavlink_serial, len+1);
        mavlink_serial[-1] = '\0';
        serialPuts(serial, mavlink_serial);
//        mavlink_msg_to_send_buffer(buf, &msg);
//        const char testing = '\0';
//        serialPutchar(serial, testing);
//        std::cout << buf << "\n";
    }
}

//void Drone::mavlink_receive_data() {
//
//    mavlink_message_t msg;
//    mavlink_status_t status;
//
//    while(serialDataAvail(serial)) {
//        uint8_t c = serialGetchar(serial);
//
////        std::cout << std::to_string(c) << '\n';
//
//
//        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
//            printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
//            // Handle message
//            switch(msg.msgid) {
//                case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
//                {
////#ifdef SOFT_SERIAL_DEBUGGING
//                    std::cout << "Ardupilot Heartbeat\n";
////#endif
//                }
//                    break;
//
//                case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
//                {
//                    /* Message decoding: PRIMITIVE
//                     *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
//                     */
//                    //mavlink_message_t* msg;
//                    mavlink_sys_status_t sys_status;
//                    mavlink_msg_sys_status_decode(&msg, &sys_status);
////#ifdef SOFT_SERIAL_DEBUGGING
//                    std::cout << "Battery voltage: " << sys_status.voltage_battery << "; current: "
//                    << sys_status.current_battery << "; comm loss: " << sys_status.drop_rate_comm << "\n";
////#endif
//                }
//                    break;
//
//                case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
//                {
//                    /* Message decoding: PRIMITIVE
//                     *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
//                     */
//                    //mavlink_message_t* msg;
//                    mavlink_param_value_t param_value;
//                    mavlink_msg_param_value_decode(&msg, &param_value);
////#ifdef SOFT_SERIAL_DEBUGGING
//                    std::cout << "Param values: " << param_value.param_value << "; count: " << param_value.param_count
//                    << "; index: " << param_value.param_index  << "; id: " << param_value.param_id << "; type:  "
//                    <<param_value.param_type << "\n";
////#endif
//                }
//                    break;
//
//                case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
//                {
//                    /* Message decoding: PRIMITIVE
//                     *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
//                     */
//                    mavlink_raw_imu_t raw_imu;
//                    mavlink_msg_raw_imu_decode(&msg, &raw_imu);
////#ifdef SOFT_SERIAL_DEBUGGING
//                    std::cout << "Raw IMU: " << raw_imu.xacc << "\n";
////#endif
//                }
//                    break;
//
//                case MAVLINK_MSG_ID_ATTITUDE:  // #30
//                {
//                    /* Message decoding: PRIMITIVE
//                     *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
//                     */
//                    mavlink_attitude_t attitude;
//                    mavlink_msg_attitude_decode(&msg, &attitude);
////#ifdef SOFT_SERIAL_DEBUGGING
//                    std::cout << "Attitude: " << attitude.roll << "\n";
//
////#endif
//                }
//                    break;
//
//                default:
//                    break;
//            }
//        }
//    }
//}

//bool Drone::mavlink_receive_response() {
//
//    mavlink_message_t msg;
//    mavlink_status_t status;
//    bool mavlink_established = false;
//
//    while(serialDataAvail(serial)) {
//        mavlink_established = true;
//        uint8_t c = serialGetchar(serial);
//
////        std::cout << std::to_string(c) << '\n';
//
//        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
//
//            // Handle message
//            switch(msg.msgid) {
//                case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
//                {
////#ifdef SOFT_SERIAL_DEBUGGING
//                    std::cout << "Ardupilot Heartbeat\n";
////#endif
//                }
//                    break;
//
//                case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
//                {
//                    /* Message decoding: PRIMITIVE
//                     *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
//                     */
//                    //mavlink_message_t* msg;
//                    mavlink_sys_status_t sys_status;
//                    mavlink_msg_sys_status_decode(&msg, &sys_status);
////#ifdef SOFT_SERIAL_DEBUGGING
//                    std::cout << "Battery voltage: " << sys_status.voltage_battery << "; current: "
//                    << sys_status.current_battery << "; comm loss: " << sys_status.drop_rate_comm << "\n";
////#endif
//                }
//                    break;
//
//                case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
//                {
//                    /* Message decoding: PRIMITIVE
//                     *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
//                     */
//                    //mavlink_message_t* msg;
//                    mavlink_param_value_t param_value;
//                    mavlink_msg_param_value_decode(&msg, &param_value);
////#ifdef SOFT_SERIAL_DEBUGGING
//                    std::cout << "Param values: " << param_value.param_value << "; count: " << param_value.param_count
//                    << "; index: " << param_value.param_index  << "; id: " << param_value.param_id << "; type:  "
//                    <<param_value.param_type << "\n";
////#endif
//                }
//                    break;
//
//                case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
//                {
//                    /* Message decoding: PRIMITIVE
//                     *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
//                     */
//                    mavlink_raw_imu_t raw_imu;
//                    mavlink_msg_raw_imu_decode(&msg, &raw_imu);
////#ifdef SOFT_SERIAL_DEBUGGING
//                    std::cout << "Raw IMU: " << raw_imu.xacc << "\n";
////#endif
//                }
//                    break;
//
//                case MAVLINK_MSG_ID_ATTITUDE:  // #30
//                {
//                    /* Message decoding: PRIMITIVE
//                     *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
//                     */
//                    mavlink_attitude_t attitude;
//                    mavlink_msg_attitude_decode(&msg, &attitude);
////#ifdef SOFT_SERIAL_DEBUGGING
//                    std::cout << "Attitude: " << attitude.roll << "\n";
//
////#endif
//                }
//                    break;
//
//                default:
//                    break;
//            }
//        }
//    }
//    return mavlink_established;
//}

DRONE_STATE Drone::get_state() {
    return state;
}

bool Drone::mavlink_positioning_status(){
//    TODO parse status response
    return true;
};

void Drone::mavlink_command_long(uint16_t command, uint8_t confirmation, float param1 = 0, float param2 = 0, float param3  = 0, float param4  = 0, float param5  = 0, float param6 = 0, float param7 = 0 ){
    mavlink_message_t msg;
    uint8_t mavlink_buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(mavlink_sys_id, mavlink_comp_id, &msg, mavlink_target_sys_id, mavlink_target_comp_id, command, confirmation, param1, param2, param3, param4, param5, param6, param7);
    mavlink_msg_to_send_buffer(mavlink_buffer, &msg);
    char mavlink_serial[MAVLINK_MAX_PACKET_LEN];
    memcpy(mavlink_buffer, mavlink_serial, MAVLINK_MAX_PACKET_LEN);
    serialPuts(serial,mavlink_serial);
}

//mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
//mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);

//void Drone::mavlink_arm() {
////    MAV_CMD_COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |0: disarm, 1: arm| 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
//
//    mavlink_message_t msg;
//    uint8_t mavlink_buffer[MAVLINK_MAX_PACKET_LEN];
//    std::cout << "Arming Drone\n";
//
////    mavlink_msg_command_long_pack(mavlink_sys_id, mavlink_comp_id, &msg, mavlink_target_sys_id, mavlink_target_comp_id, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
//    mavlink_msg_command_long_pack(10, 0, &msg, 1, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
//    mavlink_msg_to_send_buffer(mavlink_buffer, &msg);
//    char mavlink_serial[MAVLINK_MAX_PACKET_LEN];
//    memcpy(mavlink_buffer, mavlink_serial, MAVLINK_MAX_PACKET_LEN);
//    serialPuts(serial, mavlink_serial);
//}
//
//void Drone::mavlink_disarm() {
////    MAV_CMD_COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |0: disarm, 1: arm| 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
//
//    mavlink_message_t msg;
//    uint8_t mavlink_buffer[MAVLINK_MAX_PACKET_LEN];
//    std::cout << "Disarming Drone\n";
//    mavlink_msg_command_long_pack(10, 0, &msg, 1, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
//
////    mavlink_msg_command_long_pack(mavlink_sys_id, MAV_COMP_ID_ALL, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
//    mavlink_msg_to_send_buffer(mavlink_buffer, &msg);
//    char mavlink_serial[MAVLINK_MAX_PACKET_LEN];
//    memcpy(mavlink_buffer, mavlink_serial, MAVLINK_MAX_PACKET_LEN);
//    serialPuts(serial, mavlink_serial);
//}

//void Drone::mavlink_takeoff() {
////    MAV_CMD_NAV_TAKEOFF 22
////    MAV_CMD_NAV_TAKEOFF_LOCAL 24
////    mavlink_message_t msg;
////    uint8_t mavlink_buffer[MAVLINK_MAX_PACKET_LEN];
////
////    mavlink_msg_command_long_pack(mavlink_sys_id, mavlink_comp_id, &msg, 1, 1, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0);
////    uint16_t len = mavlink_msg_to_send_buffer(mavlink_buffer, &msg);
////    char *mavlink_serial;
////    memcpy(mavlink_buffer, mavlink_serial, MAVLINK_MAX_PACKET_LEN);
////    serialPuts(serial,mavlink_serial);
//    mavlink_command_long(MAV_CMD_DO_SET_MODE,0,MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,2);
//
//}

//void Drone::mavlink_set_flight_mode(FLIGHT_MODE mode) {
////    mavlink_message_t msg;
////    uint8_t mavlink_buffer[MAVLINK_MAX_PACKET_LEN];
////
////    mavlink_msg_command_long_pack(mavlink_sys_id, mavlink_comp_id, &msg, 1, 1, MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, 0, 0, 0, 0, 0);
////    uint16_t len = mavlink_msg_to_send_buffer(mavlink_buffer, &msg);
////    char *mavlink_serial;
////    memcpy(mavlink_buffer, mavlink_serial, MAVLINK_MAX_PACKET_LEN);
////    serialPuts(serial,mavlink_serial);
//    mavlink_command_long(MAV_CMD_DO_SET_MODE,0,MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode);
//}



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

int Drone::get_mode()
{
    //printf("get_mode call: %d\n", r_mode);
    return(r_mode);
}

int Drone::get_armed()
{
    //printf("get_armed call: %d\n",r_armed);
    return(r_armed);
}

int Drone::mavlink_handle_message(mavlink_message_t *msg)
{
    mavlink_heartbeat_t hb;

    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            printf("MAVLINK_MSG_ID_HEARTBEAT\n");
            mavlink_msg_heartbeat_decode(msg, &hb);
            printf("SystemID: %d\n", msg->sysid);
            printf("Component ID: %d\n", msg->compid);
            printf("status: %d\n", hb.system_status);
            printf("custom mode: %d\n", hb.custom_mode);
            printf("autopilot: %d\n", hb.autopilot);
            printf("type: %d\n", hb.type);
            //printf("base_mode: %d\n", hb.base_mode);
            printf("system_status: %d\n",hb.system_status);

            // Check the arm status:
            int armed_state = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
            printf("Armed? %s \n", armed_state ? "yes" : "no");
            //printf("raw: %d", armed_state);
            printf("\n");

            // Setup the rover parameters:
            r_mode = hb.custom_mode;
            r_armed = armed_state;
            return(1);
        }
        default:
        {
            //printf("Warning, did not handle message id %i\n",msg->msgid);
            return(-1);
        }
    }
}

int Drone::mavlink_receive_data()
{
    bool success;
    mavlink_message_t msg;

    success = serial_port.read_message(msg);

    if(success)
    {
        int id = mavlink_handle_message(&msg);
        return(id);
    }
    return(-1);
}

int Drone::mavlink_set_flight_mode(FLIGHT_MODE mode)
    {
    // Format the data:
    mavlink_command_long_t set_mode = {0};
    set_mode.target_system = 1;
    set_mode.target_component = 0;
    set_mode.command = MAV_CMD_DO_SET_MODE;		//176
    set_mode.confirmation = true;
    set_mode.param1 = 1;
    set_mode.param2 = mode;

    // Encode:
    mavlink_message_t msg;
    mavlink_msg_command_long_encode(1, 255, &msg, &set_mode);

    // Write in the serial:
    int len = serial_port.write_message(msg);
    //printf("Guided mode (%d)\n", len);
    return(len);
}

int Drone::mavlink_arm(int state)
{
    // Format the data:
    mavlink_command_long_t armed = {0};
    armed.target_system = 1;
    armed.target_component = 0;
    armed.command = MAV_CMD_COMPONENT_ARM_DISARM; //400
    armed.confirmation = true;
    armed.param1 = (int) state;

    // Encode:
    mavlink_message_t msg;
    mavlink_msg_command_long_encode(1, 255, &msg, &armed);

    // Write in the serial:
    int len = serial_port.write_message(msg);
    //printf("Rover armed = %d (%d)\n",state, len);
    return(len);
}

int Drone::mavlink_disarm() {
    return mavlink_arm(1);
}


int Drone::setAngleSpeed(float angle, float speed)
{
    //Valid input:
    if(( speed < 0.0 ) || ( speed > 1.0 ))
    {
        printf("print invalid speed input: [0-1] * cruise speed\n");
        speed = 0.0;
    }

    // Format the data:
    mavlink_command_long_t set_yawspeed = {0};
    set_yawspeed.target_system = 1;
    set_yawspeed.target_component = 0;
    set_yawspeed.command = MAV_CMD_NAV_SET_YAW_SPEED; //; 	//213 (MAV_CMD_DO_SET_POSITION_YAW_THRUST)
    set_yawspeed.confirmation = false;
    set_yawspeed.param1 = angle;			 	//angle (centridegree) [-4500 - 4500]
    set_yawspeed.param2 = speed;	 			//speed normalized [0 - 1]

    // Encode:
    mavlink_message_t msg;
    mavlink_msg_command_long_encode(1, 255, &msg, &set_yawspeed);

    // Write in the serial:
    int len = serial_port.write_message(msg);
    printf("Send angle: %f | speed: %f\n", angle, speed);
    return(len);
}
