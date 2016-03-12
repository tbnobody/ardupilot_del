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

#ifndef __AP_HOTT_ALARM_H__
#define __AP_HOTT_ALARM_H__

#include <string.h>
#include <inttypes.h>

#define HOTT_ALARM_QUEUE_MAX 5

#define HOTT_ALARM_NUM(a) (a - 0x40)

class AP_HoTT_Alarm {
public:
    struct HoTT_Alarm_Event_t {
        uint16_t alarm_time;        // Alarm play time in 1sec units
        uint16_t alarm_time_replay; // Alarm repeat time in 1sec. units. 0 -> One time alarm
                                    // forces a delay between new alarms of the same kind
        uint8_t visual_alarm1;      // Visual alarm bitmask
        uint8_t visual_alarm2;      // Visual alarm bitmask
        uint8_t alarm_num;          // Alarm number 0..255 (A-Z)
        uint8_t alarm_profile;      // profile id ie EAM_SENSOR_ID
    };

    // constructor
    AP_HoTT_Alarm();

    // add - adds an alarm to active queue
    bool add(HoTT_Alarm_Event_t *alarm);

    uint8_t get_alarm_for_profile_id(uint8_t hott_profile_id, HoTT_Alarm_Event_t &e);

    // scheduler - active alarm scheduler
    //  should be called every second
    void scheduler(void);

    // update_replay_queue - updates replay delay queue
    //  should be called every second
    void update_replay_queue(void);

private:
    // add_replay - adds an alarm to replay queue
    void add_replay(HoTT_Alarm_Event_t *alarm);

    // exists - checks if an alarm exists
    bool exists(HoTT_Alarm_Event_t *alarm);

    // active_exists - checks if an alarm exists in active queue
    bool active_exists(HoTT_Alarm_Event_t *alarm);

    // replay_exists - checks if an alarm exists in replay queue
    bool replay_exists(HoTT_Alarm_Event_t *alarm);

    // remove - removes an alarm from active queue
    //  first alarm at offset 1
    void remove(uint8_t num);

    // remove_replay - removes an alarm from replay queue
    //  first alarm at offset 1
    void remove_replay(uint8_t num);

    uint8_t _alarm_cnt;
    uint8_t _active_alarm; //  Current active voice alarm number
    uint8_t _alarm_replay_cnt;

    HoTT_Alarm_Event_t _alarm_queue[HOTT_ALARM_QUEUE_MAX];
    HoTT_Alarm_Event_t _alarm_replay_queue[HOTT_ALARM_QUEUE_MAX];
};
#endif