
#include "mavlink/ardupilotmega/mavlink.h"

mavlink_system_t mavlink_system = {
        1, // System ID (1-255)
        1  // Component ID (a MAV_COMPONENT value)
};

// set this to the hardware serial port you wish to use
#define PIXHAWK   Serial2

// to ask pixhawk again for the datastreams every once in a while
int resendDataRequistTimerStart = 0;
int resendDataRequistInterval = 3000;

void setup() {
    Serial.begin(250000);
    PIXHAWK.begin(57600);
}

void comm_receive() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while(PIXHAWK.available()>0) {
        uint8_t c = PIXHAWK.read();
        // Try to get a new message
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

            // Handle message
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
                {
                    // E.g. read GCS heartbeat and go into
                    // comm lost mode if timer times out
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg,&hb);
                    Serial.println("Received: HEARTBEAT");
                    Serial.println(hb.mavlink_version);
                    Serial.println("");
                }
                    break;

                default:
                    Serial.println("Received: message of unhandled type");
                    break;
            }
        }
    }
}


void Mav_Request_Data()
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // To be setup according to the needed information to be requested from the Pixhawk
    const int  maxStreams = 2;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_ALL, MAV_DATA_STREAM_EXTRA1};
    const uint16_t MAVRates[maxStreams] = {0x02,0x05};

    for (int i=0; i < maxStreams; i++) {
        mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        PIXHAWK.write(buf, len);
    }
}

void loop() {
    comm_receive();

    // ask again for the datastreams every once in a while
    if(millis() - resendDataRequistTimerStart > resendDataRequistInterval)
    {
        Mav_Request_Data();
        resendDataRequistTimerStart = millis();
    }
}


void Command_long_DISARM(){
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(1, 1, &msg, 1, 1, 400, 0, 0, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    PIXHAWK.write(buf, len);
}
//
// Created by ColinLaganier on 25/04/2022.
//

