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

/* 
   HoTT Alarm library
*/
#include "AP_HoTT_Alarm.h"

AP_HoTT_Alarm::AP_HoTT_Alarm():
	  _alarmCnt(0),
	  _activeAlarm(0),
	  _alarm_ReplayCnt(0)
    {}
    
    
bool AP_HoTT_Alarm::add(struct _hott_alarm_event_T *alarm)
{
    if(alarm == 0)
        return false;

    if(_alarmCnt >= HOTT_ALARM_QUEUE_MAX)
        return false;   // no more space left...

    if(exists(alarm)) 
        return false;

    // we have a new alarm
    memcpy(&_alarm_queue[_alarmCnt++], alarm, sizeof(struct _hott_alarm_event_T));
    return true;
}

uint8_t AP_HoTT_Alarm::getAlarmForProfileId(uint8_t hottProfileId, _hott_alarm_event &e)
{
    if(_alarmCnt == 0)
        return 0;

    uint8_t alarmCount = 0;
    for(int i = 0; i <  _alarmCnt; i++) {
        if(_alarm_queue[i].alarm_profile == hottProfileId) {
            e.visual_alarm1 |= _alarm_queue[i].visual_alarm1;
            e.visual_alarm2 |= _alarm_queue[i].visual_alarm2;
            if(i == _activeAlarm - 1) {
                //this alarm is also active
                e.alarm_num = _alarm_queue[i].alarm_num;
            }
            alarmCount++;
        }
    }
    return alarmCount;
}

void AP_HoTT_Alarm::scheduler(void)
{
    static uint8_t activeAlarmTimer = 3 * 50;

    if(_alarmCnt < 1)
        return; // no alarms 
    
    for(uint8_t i = 0; i < _alarmCnt; i++) {
        if(_alarm_queue[i].alarm_time == 0) {
            // end of alarm, remove it
            if(_alarm_queue[i].alarm_time_replay != 0)
                add_replay(&_alarm_queue[i]);
            remove(i + 1);    // first alarm at offset 1
            --i;    // correct counter
            continue;
        }
    }

    if(_activeAlarm != 0) { // is an alarm active
        if (++activeAlarmTimer % 2 == 0) {    // every 1sec
            _alarm_queue[_activeAlarm - 1].alarm_time--;
        }
        if (activeAlarmTimer < 50 * 2) // alter alarm every 2 sec
            return;
    }
    activeAlarmTimer = 0;

    if(++_activeAlarm > _alarmCnt) {
        _activeAlarm = 1;
    }
    if(_alarmCnt <= 0) {
        _activeAlarm = 0;
        return;
    }
}

void AP_HoTT_Alarm::add_replay(struct _hott_alarm_event_T *alarm)
{
    if(alarm == 0)
        return;
    if(_alarm_ReplayCnt >= HOTT_ALARM_QUEUE_MAX)
        return; //no more space left...
    if(replay_exists(alarm)) 
        return;
    // we have a new alarm
    memcpy(&_alarm_replay_queue[_alarm_ReplayCnt++], alarm, sizeof(struct _hott_alarm_event_T));
}	

bool AP_HoTT_Alarm::exists(struct _hott_alarm_event_T *alarm)
{
    if(active_exists(alarm))
        return true;

    if(replay_exists(alarm))
        return true;

    return false;
}

bool AP_HoTT_Alarm::active_exists(struct _hott_alarm_event_T *alarm)
{
    // check active alarms
    for(uint8_t i = 0; i < _alarmCnt; i++) {
        if(_alarm_queue[i].alarm_num == alarm->alarm_num &&
            _alarm_queue[i].alarm_profile == alarm->alarm_profile) {
            // alarm exists.
            return true;
        }
    }
    return false;
}

bool AP_HoTT_Alarm::replay_exists(struct _hott_alarm_event_T *alarm)
{
    // check replay delay queue
    for(uint8_t i = 0; i < _alarm_ReplayCnt; i++) {
        if(_alarm_replay_queue[i].alarm_num == alarm->alarm_num &&
            _alarm_replay_queue[i].alarm_profile == alarm->alarm_profile) {
            // alarm exists
            return true;
        }
    }
    return false;
}

void AP_HoTT_Alarm::remove(uint8_t num)
{
    if(num > _alarmCnt || num == 0)    // has to be > 0
        return; // possibile error

    if(_alarmCnt != 1) {
        memcpy(&_alarm_queue[num-1], &_alarm_queue[num], sizeof(struct _hott_alarm_event_T) * (_alarmCnt - num) );
    }
    --_alarmCnt;
}

void AP_HoTT_Alarm::remove_replay(uint8_t num)
{
    if(num > _alarm_ReplayCnt || num == 0) // has to be > 0
        return; // possibile error

    if(_alarm_ReplayCnt != 1) {
        memcpy(&_alarm_replay_queue[num - 1], &_alarm_replay_queue[num], sizeof(struct _hott_alarm_event_T) * (_alarm_ReplayCnt - num) );
    }
    --_alarm_ReplayCnt;
}

void AP_HoTT_Alarm::update_replay_queue(void)
{
    for(uint8_t i = 0; i <  _alarm_ReplayCnt; i++) {
        if(--_alarm_replay_queue[i].alarm_time_replay == 0) {
            //remove it
            remove_replay(i + 1);
            i--;
            continue;
        }
    }
}