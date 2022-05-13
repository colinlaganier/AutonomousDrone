/* MAVLInk_DroneLights
 *  by Juan Pedro López
 *
 * This program was developed to connect an Arduino board with a Pixhawk via MAVLink
 *   with the objective of controlling a group of WS2812B LED lights on board of a quad
 *
 * The current version of the program is working properly.
 *
 * TO DO:
 *  - Move STREAMS request to RC_CHANNELS to use values in logic
 *  - Add RC_CHANNLES_RAW messages monitoring: move #30 to RC_CHANNELS_RAW (#35)
 *      http://mavlink.org/messages/common#RC_CHANNELS_RAW
 *  - Look for message on low battery:
 *      To be tested: http://mavlink.org/messages/common#PARAM_REQUEST_READ
 *      To be checked: http://mavlink.org/messages/common#SYS_STATUS
 *  - Potential implementation of other alarms, like high intensity
 *
 * You can restrict the maximum package size with this parameter in mavlink_types.h:

    #ifndef MAVLINK_MAX_PAYLOAD_LEN_
    // it is possible to override this, but be careful! Defa_
    #define **MAVLINK_MAX_PAYLOAD_LEN 255 ///< Maximum payload length_
    #endif_
 */


// In case we need a second serial port for debugging
#define SOFT_SERIAL_DEBUGGING   // Comment this line if no serial debugging is needed
#ifdef SOFT_SERIAL_DEBUGGING
// Library to use serial debugging with a second board
#include <SoftwareSerial.h>
SoftwareSerial mySerial(11, 12); // RX, TX
SoftwareSerial pxSerial(9,10);   // RX, TX
#endif

#include "FastLED.h"

#include "mavlink.h"
//#include "common/mavlink_msg_request_data_stream.h"


// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

// FastLed setup
// How many leds in your strip?
#define NUM_LEDS 8
#define BRIGHTNESS  255  // Put 0 to switch off all leds


// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 2
//#define MODE_PIN 3
//#define CLOCK_PIN 13

// Define the array of leds
CRGB leds[NUM_LEDS];

// Status: 0, 2, 4 for lights off, 1, 3 for lights on
// Pulses: flashing with two 20 ms pulses, 80 ms in between pulses and 1 Hz frequency
int leds_status = 0;

// Light modes: 1, normal mode. 0, off. 2, low battery.
#define NUM_MODOS 3
int leds_modo = 1;

// Definición de las matrices ON y OFF de los modos
CRGB led_on[NUM_MODOS][NUM_LEDS] = {
        // Mode 0
        {
                CRGB::White,   // Warning: test values
                CRGB::Black,
                CRGB::Black,
                CRGB::Black,
                CRGB::Black,
                CRGB::Black,
                CRGB::Black,
                CRGB::Black
        },
        // Mode 1
        {
                CRGB::White,  // Warning: test values
                CRGB::White,
                CRGB::Red,
                CRGB::Green,
                CRGB::White,
                CRGB::White,
                CRGB::Red,
                CRGB::Red
        },
        // Mode 2
        {
                CRGB::Blue,  // Warning: test values
                CRGB::White,
                CRGB::Red,
                CRGB::Green,
                CRGB::White,
                CRGB::White,
                CRGB::Red,
                CRGB::Red
        }
};

CRGB led_off[NUM_MODOS][NUM_LEDS] = {
        // Mode 0
        {
                CRGB::Blue,    // Static
                CRGB::Black,   // Static
                CRGB::Black,   // Static
                CRGB::Black,   // Static
                CRGB::Black,   // Pulses
                CRGB::Black,   // Pulses
                CRGB::Black,   // Pulses
                CRGB::Black    // Pulses
        },
        // Mode 1
        {
                CRGB::Black,   // Static
                CRGB::White,   // Static
                CRGB::Red,     // Static
                CRGB::Green,   // Static
                CRGB::Black,   // Pulses
                CRGB::Black,   // Pulses
                CRGB::Black,   // Pulses
                CRGB::Black    // Pulses
        },
        // Mode 2
        {
                CRGB::Black,   // Static
                CRGB::White,   // Static
                CRGB::Black,   // Static
                CRGB::Black,   // Static
                CRGB::Black,   // Pulses
                CRGB::Black,   // Pulses
                CRGB::Black,   // Pulses
                CRGB::Black    // Pulses
        }
};


// Lights flashing adjustment
unsigned long previousMillis = 0;     // will store last time LED was updated
unsigned long next_interval = 0;      // next interval
const long tiempo_on = 20;
const long tiempo_off = 80;
const long tiempo_descanso = 880;
int test_led_tipo = 4;


void setup() {
    // MAVLink interface start
    Serial.begin(57600);

    // LEDs setup
    //delay(5000); // sanity delay
    FastLED.addLeds<WS2811, DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness( BRIGHTNESS );

#ifdef SOFT_SERIAL_DEBUGGING
    // [DEB] Soft serial port start
    Serial.begin(57600);
    Serial.println("MAVLink starting.");
    mySerial.begin(57600);
#endif
}

void loop() {
    // Lights management
    // Light pulses: 2 quick flashes per second. 100 ms each cycle
    unsigned long currentMillis = millis();
    int i=0;


    // MAVLink
    /* The default UART header for your MCU */
    int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
    int compid = 158;                ///< The component sending the message
    int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

    // Define the system type, in this case an airplane -> on-board controller
    // uint8_t system_type = MAV_TYPE_FIXED_WING;
    uint8_t system_type = MAV_TYPE_GENERIC;
    uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

    uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
    uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    unsigned long currentMillisMAVLink = millis();
    if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
        previousMillisMAVLink = currentMillisMAVLink;

#ifdef SOFT_SERIAL_DEBUGGING
        pxSerial.write(buf,len);
        //mySerial.println("Ardu HB");
#else
        Serial.write(buf, len);
#endif

        //Mav_Request_Data();
        num_hbs_pasados++;
        if(num_hbs_pasados>=num_hbs) {
            // Request streams from Pixhawk
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("Streams requested!");
#endif
            Mav_Request_Data();
            num_hbs_pasados=0;
        }

    }

    // Check reception buffer
    comm_receive();
}

void Mav_Request_Data()
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];


    // STREAMS that can be requested
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
    const int  maxStreams = 2;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
    const uint16_t MAVRates[maxStreams] = {0x02,0x05};


    for (int i=0; i < maxStreams; i++) {
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
#ifdef SOFT_SERIAL_DEBUGGING
        pxSerial.write(buf,len);
#else
        Serial.write(buf, len);
#endif
    }

    // Request: PARAM_REQUEST_LIST. Only for full log recording
    /*
     * Primitive: mavlink_msg_param_request_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                     uint8_t target_system, uint8_t target_component)
     */
/*
  // Configure
  uint8_t system_id=2;
  uint8_t component_id=200;
  // mavlink_message_t* msg;
  uint8_t target_system=1;
  uint8_t target_component=0;

  // Pack
  mavlink_msg_param_request_list_pack(system_id, component_id, &msg,
    target_system, target_component);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send
#ifdef SOFT_SERIAL_DEBUGGING
    pxSerial.write(buf,len);
#else
    Serial.write(buf, len);
#endif
*/
}



void comm_receive() {

    mavlink_message_t msg;
    mavlink_status_t status;

    // Echo for manual debugging
    // Serial.println("---Start---");

#ifdef SOFT_SERIAL_DEBUGGING
    while(pxSerial.available()>0) {
        uint8_t c = pxSerial.read();
#else
        while(Serial.available()>0) {
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