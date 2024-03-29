// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __AP_HOTT_TELEM_H__
#define __AP_HOTT_TELEM_H__

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_Compass/AP_Compass.h>
#include "AP_HoTT_Alarm.h"
#include "AP_HoTT_TextMode.h"
#include "hott_msgs.h"

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include "../../ArduCopter/defines.h"
#endif

class AP_HoTT_Telem {
public:
    //constructor
    AP_HoTT_Telem(AP_AHRS &ahrs, AP_BattMonitor &battery, Location &current_loc, AP_Baro &barometer);

    // init - perform require initialisation including detecting which protocol to use
    void init(const AP_SerialManager& serial_manager);

    // update_data - updates structs containing telemetry information
    void update_data(uint8_t control_mode, uint32_t wp_distance, int32_t wp_bearing, int32_t home_distance, int32_t home_bearing, bool armed);

private:
    // init_uart - initialise uart
    void init_uart();

    // hott_tick - main call to send updates to transmitter
    //  called by scheduler at a high rate
    void hott_tick();

    // send_data - sends data from specified struct to uart
    void send_data(uint8_t *buffer, size_t current_msg_size);

    // update_gps_data - writes data into _hott_gps_msg struct
    void update_gps_data();

    // update_eam_data - writes data into _hott_eam_msg struct
    void update_eam_data();

    // update_vario_data - writes data into _hott_vario_msg struct
    void update_vario_data();

    // convert_lat_long
    void convert_lat_long(float degree, uint8_t &pos_ns_ew, uint16_t &deg_minutes, uint16_t &deg_seconds);

    // processClimbrate - calculates and maintans climbrate changes
    //  called every 1s with current altitude as input
    void process_climbrate(int16_t current_altitude);

    // eam_check_mah - check for used mAh
    void eam_check_mah(void);

    // eam_check_main_power - check for low batteries
    void eam_check_main_power(void);

    // get_altitude_rel - get relative altitude from ground
    uint16_t get_altitude_rel();

    AP_AHRS &_ahrs;                         // reference to attitude estimate
    AP_BattMonitor &_battery;               // reference to battery monitor object
    Location &_current_loc;                 // reference to current_loc object
    AP_Baro &_barometer;                    // reference to barometer object
    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    bool _initialised_uart;                 // true when we have detected the protocol and UART has been initialised

    uint8_t _mode;
    uint32_t _wp_distance;
    int32_t _wp_bearing;
    int32_t _home_distance;
    int32_t _home_bearing;
    bool _armed;

    uint32_t _current_delay_ms;
    uint32_t _last_delay_ms;
    uint32_t _last_delay_1s;

    enum hott_status {
        HOTT_IDLE,
        HOTT_RCV_MODE_BINARY,
        HOTT_RCV_MODE_TEXT,
        HOTT_RCV_ID,
        HOTT_SEND_GPS,
        HOTT_SEND_EAM,
        HOTT_SEND_VARIO,
        HOTT_SEND_TEXT
    };

    hott_status _hott_status;

    struct HOTT_GPS_MSG _hott_gps_msg;
    struct HOTT_EAM_MSG _hott_eam_msg;
    struct HOTT_VARIO_MSG _hott_vario_msg;
    struct HOTT_TEXTMODE_MSG _hott_text_msg;

    size_t _current_msg_pos;
    uint16_t _checksum;

    // for vario calculations
    int _climbrate1s;
    int _climbrate3s;
    int _climbrate10s;

    // electrical time
    uint32_t _electric_time;  //time in ARMED mode in seconds

    AP_HoTT_Alarm _alarms;
    AP_HoTT_TextMode _text_mode;

};
#endif