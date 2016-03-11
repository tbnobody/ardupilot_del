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

class AP_HoTT_Alarm
{
public:
    struct _hott_alarm_event_T {
        uint16_t alarm_time;        // Alarm play time in 1sec units
        uint16_t alarm_time_replay; // Alarm repeat time in 1sec. units. 0 -> One time alarm
                                    // forces a delay between new alarms of the same kind
        uint8_t visual_alarm1;      // Visual alarm bitmask
        uint8_t visual_alarm2;      // Visual alarm bitmask
        uint8_t alarm_num;          // Alarm number 0..255 (A-Z)
        uint8_t alarm_profile;      // profile id ie EAM_SENSOR_ID
    };
    typedef struct _hott_alarm_event_T _hott_alarm_event;

    //constructor
    AP_HoTT_Alarm();
    
    bool add(struct _hott_alarm_event_T *alarm);
    
    uint8_t getAlarmForProfileId(uint8_t hottProfileId, _hott_alarm_event &e);

    void scheduler(void);
    
    void update_replay_queue(void);

private:
    void add_replay(struct _hott_alarm_event_T *alarm);
    
    bool exists(struct _hott_alarm_event_T *alarm);

    bool active_exists(struct _hott_alarm_event_T *alarm);
    
    bool replay_exists(struct _hott_alarm_event_T *alarm);
    
    void remove(uint8_t num);
    
    void remove_replay(uint8_t num);
    
    uint8_t _alarmCnt;
    
    uint8_t _activeAlarm; //  Current active voice alarm number
    
    uint8_t _alarm_ReplayCnt;
    
    _hott_alarm_event _alarm_queue[HOTT_ALARM_QUEUE_MAX];
    _hott_alarm_event _alarm_replay_queue[HOTT_ALARM_QUEUE_MAX];
};
#endif