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
        HAL_Print("\nConfiguration succeeded.\n\n");
        return true;
    }
}

//void DWM1001C::dwm_get_position() {
//
//    vehicle_position_msg msg;
//
//    // Coordinate verification
//    if (loc.p_pos->x == 0 || loc.p_pos->y == 0) {
//        return;
//    }
//
//    msg.info.x = loc.p_pos->x;
//    msg.info.y = loc.p_pos->y;
//    //msg.info.z = loc.p_pos->z;
//    msg.info.z = 0;
//    msg.info.position_error = loc.p_pos->qf;
//    send_message(MSGID_POSITION, sizeof(msg.buf), msg.buf);
//}

//int DWM1001C::setup_serial() {
//    // Initializing UART communication
//    uwb_serial = serialOpen("/dev/ttyUSB0", 115200);
//    if (uwb_serial < 0)
//    {
//        std::cout << "Unable to open uwb serial device\n";
//        return 1 ;
//    }
//    if (wiringPiSetup() == -1)
//    {
//        std::cout << "Unable to start uwb wiringPi\n";
//        return 1 ;
//    }
////    char init_message[] = {"This is the init message"};
//    serialPuts(uwb_serial,init_message);
//    return 0;
//}

//void DWM1001C::send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[])
//{
//    // sanity check
//    if (data_len == 0) {
//        return;
//    }
//
//    // message is buffer length + 1 (for checksum)
//    uint8_t msg_len = data_len+1;
//
//    // calculate checksum and place in last element of array
//    uint8_t checksum = 0;
//    checksum ^= msg_id;
//    checksum ^= msg_len;
//    for (uint8_t i=0; i<data_len; i++) {
//        checksum = checksum ^ data_buf[i];
//    }
//
//    // send message
//    int16_t num_sent = 0;
//
//    num_sent += fcboardSerial.write(MSG_HEADER);
//    num_sent += fcboardSerial.write(msg_id);
//    num_sent += fcboardSerial.write(msg_len);
//    num_sent += fcboardSerial.write(data_buf, data_len);
//    num_sent += fcboardSerial.write(&checksum, 1);
//    fcboardSerial.flush();
//}
