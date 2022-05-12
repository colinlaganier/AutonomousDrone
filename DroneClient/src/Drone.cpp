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

int Drone::setup_serial(int *serial) {
    // Initializing UART communication
    *serial = serialOpen("/dev/ttyAMA0", 115200);
    if (*serial < 0)
    {
        std::cout << "Unable to open serial device\n";
        return 1 ;
    }
    if (wiringPiSetup() == -1)
    {
        std::cout << "Unable to start wiringPi\n";
        return 1 ;
    }
}

void Drone::setup_mavlink() {

}

void mavlink_request_data(int *serial) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    /*
     * Definitions are in common.h: enum MAV_DATA_STREAM
     *
     * MAV_DATA_STREAM_ALL=0, // Enable all data streams
     * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
     * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
     * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
     * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
     * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
     * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
     * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
     * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
     * MAV_DATA_STREAM_ENUM_END=13,
     *
     * Data in PixHawk available in:
     *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
     *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
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
        mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        serialPutchar(*serial, buf);
        std::cout << buf << "\n";
    }
}

void mavlink_receive_data(int *serial) {

    mavlink_message_t msg;
    mavlink_status_t status;

    // Echo for manual debugging
    // Serial.println("---Start---");

#ifdef SOFT_SERIAL_DEBUGGING
    while(pxSerial.available()>0) {
        uint8_t c = pxSerial.read();
#else
    while(serialDataAvail(*serial)) {
        uint8_t c = Serial.read();
#endif

        // Try to get a new message
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

            // Handle message
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
                {
                    // E.g. read GCS heartbeat and go into
                    // comm lost mode if timer times out
#ifdef SOFT_SERIAL_DEBUGGING
                    //mySerial.println("PX HB");
#endif
                }
                    break;

                case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
                {
                    /* Message decoding: PRIMITIVE
                     *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
                     */
                    //mavlink_message_t* msg;
                    mavlink_sys_status_t sys_status;
                    mavlink_msg_sys_status_decode(&msg, &sys_status);
#ifdef SOFT_SERIAL_DEBUGGING
                    mySerial.print("PX SYS STATUS: ");
                    mySerial.print("[Bat (V): ");
                    mySerial.print(sys_status.voltage_battery);
                    mySerial.print("], [Bat (A): ");
                    mySerial.print(sys_status.current_battery);
                    mySerial.print("], [Comms loss (%): ");
                    mySerial.print(sys_status.drop_rate_comm);
                    mySerial.println("]");
#endif
                }
                    break;

                case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
                {
                    /* Message decoding: PRIMITIVE
                     *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
                     */
                    //mavlink_message_t* msg;
                    mavlink_param_value_t param_value;
                    mavlink_msg_param_value_decode(&msg, &param_value);
#ifdef SOFT_SERIAL_DEBUGGING
                    mySerial.println("PX PARAM_VALUE");
                    mySerial.println(param_value.param_value);
                    mySerial.println(param_value.param_count);
                    mySerial.println(param_value.param_index);
                    mySerial.println(param_value.param_id);
                    mySerial.println(param_value.param_type);
                    mySerial.println("------ Fin -------");
#endif
                }
                    break;

                case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
                {
                    /* Message decoding: PRIMITIVE
                     *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
                     */
                    mavlink_raw_imu_t raw_imu;
                    mavlink_msg_raw_imu_decode(&msg, &raw_imu);
#ifdef SOFT_SERIAL_DEBUGGING
                    //mySerial.println("PX RAW IMU");
                    //mySerial.println(raw_imu.xacc);
#endif
                }
                    break;

                case MAVLINK_MSG_ID_ATTITUDE:  // #30
                {
                    /* Message decoding: PRIMITIVE
                     *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
                     */
                    mavlink_attitude_t attitude;
                    mavlink_msg_attitude_decode(&msg, &attitude);
#ifdef SOFT_SERIAL_DEBUGGING
                    //mySerial.println("PX ATTITUDE");
                    //mySerial.println(attitude.roll);
                    if(attitude.roll>1) leds_modo = 0;
                    else if(attitude.roll<-1) leds_modo = 2;
                    else leds_modo=1;
#endif
                }
                    break;


                default:
#ifdef SOFT_SERIAL_DEBUGGING
                    mySerial.print("--- Otros: ");
                    mySerial.print("[ID: ");
                    mySerial.print(msg.msgid);
                    mySerial.print("], [seq: ");
                    mySerial.print(msg.seq);
                    mySerial.println("]");
#endif
                    break;
            }
        }
    }
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

