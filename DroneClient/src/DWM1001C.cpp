/**
  **********************************************************************************
  * @file     DWM1001C.cpp
  * @author   Colin Laganier
  * @version  V0.1
  * @date     2022-05-01
  * @brief   This file contains the header information for the DWM1001C positioning.
  **********************************************************************************
  * @attention  Requires DWM1001 API library & WiringPi for hardware communication.
  */

//Include Files
#include "DWM1001C.h"

/**************************************************************************************************************/
/*                                                CONSTRUCTORS                                                */
/**************************************************************************************************************/

DWM1001C::DWM1001C() {
    loc.p_pos = &pos;
    uwb_last_sensing = 0;
}

DWM1001C::~DWM1001C() {

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

void DWM1001C::dwm_setup(){
    HAL_Print("dwm_init(): dev%d\n", HAL_DevNum());
    dwm_init();

    dwm_set_params(&cfg_tag);

    HAL_Print("Wait 2s for node to reset.\n");
    HAL_Delay(2000);
    dwm_cfg_get(&cfg_node);

//    dwm_verify_config(&cfg_tag, &cfg_node);
}

void DWM1001C::dwm_set_params(dwm_cfg_tag_t *cfg_tag){
    HAL_Print("Setting to tag: dev%d.\n", HAL_DevNum());
    cfg_tag->low_power_en = 0;
    cfg_tag->meas_mode = DWM_MEAS_MODE_TWR;
    cfg_tag->loc_engine_en = 1;
    cfg_tag->common.led_en = 1;
    cfg_tag->common.ble_en = 1;
    cfg_tag->common.uwb_mode = DWM_UWB_MODE_ACTIVE;
    cfg_tag->common.fw_update_en = 0;
    HAL_Print("dwm_cfg_tag_set(&cfg_tag): dev%d.\n", HAL_DevNum());
    dwm_cfg_tag_set(cfg_tag);
}

bool DWM1001C::dwm_verify_config(){
    HAL_Print("Comparing set vs. get: dev%d.\n", HAL_DevNum());
    if((cfg_tag.low_power_en        != cfg_node.low_power_en)
       || (cfg_tag.meas_mode           != cfg_node.meas_mode)
       || (cfg_tag.loc_engine_en       != cfg_node.loc_engine_en)
       || (cfg_tag.common.led_en       != cfg_node.common.led_en)
       || (cfg_tag.common.ble_en       != cfg_node.common.ble_en)
       || (cfg_tag.common.uwb_mode     != cfg_node.common.uwb_mode)
       || (cfg_tag.common.fw_update_en != cfg_node.common.fw_update_en))
    {
        HAL_Print("low_power_en        cfg_tag=%d : cfg_node=%d\n", cfg_tag.low_power_en,     cfg_node.low_power_en);
        HAL_Print("meas_mode           cfg_tag=%d : cfg_node=%d\n", cfg_tag.meas_mode,        cfg_node.meas_mode);
        HAL_Print("loc_engine_en       cfg_tag=%d : cfg_node=%d\n", cfg_tag.loc_engine_en,    cfg_node.loc_engine_en);
        HAL_Print("common.led_en       cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.led_en,    cfg_node.common.led_en);
        HAL_Print("common.ble_en       cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.ble_en,    cfg_node.common.ble_en);
        HAL_Print("common.uwb_mode     cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.uwb_mode,  cfg_node.common.uwb_mode);
        HAL_Print("common.fw_update_en cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.fw_update_en, cfg_node.common.fw_update_en);
        HAL_Print("\nConfiguration failed.\n\n");
        return false;
    }
    else
    {
        HAL_Print("low_power_en        cfg_tag=%d : cfg_node=%d\n", cfg_tag.low_power_en,     cfg_node.low_power_en);
        HAL_Print("meas_mode           cfg_tag=%d : cfg_node=%d\n", cfg_tag.meas_mode,        cfg_node.meas_mode);
        HAL_Print("loc_engine_en       cfg_tag=%d : cfg_node=%d\n", cfg_tag.loc_engine_en,    cfg_node.loc_engine_en);
        HAL_Print("common.led_en       cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.led_en,    cfg_node.common.led_en);
        HAL_Print("common.ble_en       cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.ble_en,    cfg_node.common.ble_en);
        HAL_Print("common.uwb_mode     cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.uwb_mode,  cfg_node.common.uwb_mode);
        HAL_Print("common.fw_update_en cfg_tag=%d : cfg_node=%d\n", cfg_tag.common.fw_update_en, cfg_node.common.fw_update_en);
        HAL_Print("\nConfiguration succeeded.\n\n");
        return true;
    }
}

void DWM1001C::dwm_get_position() {

    coordinates_t position;

    if(dwm_loc_get(&(loc)) == RV_OK) {

        position.x = loc.p_pos->x;
        position.y = loc.p_pos->y;
        position.z = loc.p_pos->z;
        position.position_error = loc.p_pos->qf;
        HAL_Print("\t[%d,%d,%d,%u]\n", loc.p_pos->x, loc.p_pos->y, loc.p_pos->z,loc.p_pos->qf);

        dwm_send_vehicle_position(position);

    } else {
        std::cout << "Failed to fetch TAG position\n";
    }
}

void DWM1001C::dwm_send_vehicle_position(coordinates_t& position)
{
    vehicle_position_msg msg;

    // sanity check position
    if (position.x == 0 || position.y == 0) {
        return;
    }

    msg.info.x = position.x;
    msg.info.y = position.y;
    //msg.info.z = position.z;
    msg.info.z = 0;
    msg.info.position_error = (position.position_error == 0) ? 250 : (position.position_error * 10);
    dwm_send_message(MSGID_POSITION, sizeof(msg.buf), msg.buf);
}

int DWM1001C::dwm_setup_serial() {
    // Initializing UART communication
    uwb_serial = serialOpen("/dev/ttyUSB0", 115200);
    if (uwb_serial < 0)
    {
        std::cout << "Unable to open uwb serial device\n";
        return 1 ;
    }
    if (wiringPiSetup() == -1)
    {
        std::cout << "Unable to start uwb wiringPi\n";
        return 1 ;
    }
    return 0;
}

// send all beacon config to ardupilot
void DWM1001C::dwm_send_beacon_config()
{
    beacon_config_msg msg;
    msg.info.beacon_count = NUM_ANCHORS;
    for (uint8_t i=0; i<NUM_ANCHORS; i++) {
        msg.info.beacon_id = i;
        msg.info.x = anchors_x[i];
        msg.info.y = anchors_y[i];
        msg.info.z = anchors_z[i];
        dwm_send_message(MSGID_BEACON_CONFIG, sizeof(msg.buf), msg.buf);
    }
    std::cout << "Sent anchor info\n";
}

void DWM1001C::dwm_send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[])
{
    // sanity check
    if (data_len == 0) {
        return;
    }

    // message is buffer length + 1 (for checksum)
    uint8_t msg_len = data_len+1;

    // calculate checksum and place in last element of array
    uint8_t checksum = 0;
    checksum ^= msg_id;
    checksum ^= msg_len;
    for (uint8_t i=0; i<data_len; i++) {
        checksum = checksum ^ data_buf[i];
    }

    // send message
    int16_t num_sent = 0;

    serialPutchar(uwb_serial,MSG_HEADER);
    serialPutchar(uwb_serial,msg_id);
    serialPutchar(uwb_serial,msg_len);
    for (int i = 0; i < data_len; i++)
        serialPutchar(uwb_serial,*(data_buf + i));
    serialPutchar(uwb_serial,checksum);

}

// get ranges for each anchor
void DWM1001C::dwm_get_ranges()
{
    bool success = false;

    if(dwm_loc_get(&(loc)) == RV_OK) {
        for (uint8_t i = 0; i < loc.anchors.dist.cnt; ++i)
        {
            HAL_Print("\t%u)", i);
            HAL_Print("0x%llx", loc.anchors.dist.addr[i]);
            HAL_Print("=%u,%u\n", loc.anchors.dist.dist[i], loc.anchors.dist.qf[i]);
            dwm_send_beacon_distance(i, loc.anchors.dist.dist[i]);
        }
        success = true;
    }
    if (!success) {
        std::cout << "Failed to get ranges\n";
    }
}

// send a beacon's distance to ardupilot
void DWM1001C::dwm_send_beacon_distance(uint8_t beacon_id, uint32_t distance_mm)
{
    beacon_distance_msg msg;
    msg.info.beacon_id = beacon_id;
    msg.info.distance = distance_mm;
    dwm_send_message(MSGID_BEACON_DIST, sizeof(msg.buf), msg.buf);
}
