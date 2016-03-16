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

#ifndef __AP_HOTT_TEXTMODE_H__
#define __AP_HOTT_TEXTMODE_H__

#include <string.h>
#include <inttypes.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "hott_msgs.h"

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#define HOTT_NAME "APM:Copter"
#endif

class AP_HoTT_TextMode {
public:
    // constructor
    AP_HoTT_TextMode(HOTT_TEXTMODE_MSG &msg);

    void handle(uint8_t address);

private:
    void clear_screen();

    void print_word(uint8_t pos, char const *text, bool inverted);

    HOTT_TEXTMODE_MSG &_msg;

};
#endif