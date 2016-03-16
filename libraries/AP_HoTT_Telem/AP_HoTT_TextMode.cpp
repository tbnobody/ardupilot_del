// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*

   Inspired by work done here

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
   HoTT TextMode library
*/
#include "AP_HoTT_TextMode.h"

AP_HoTT_TextMode::AP_HoTT_TextMode(HOTT_TEXTMODE_MSG &msg):
    _msg(msg)
{}

void AP_HoTT_TextMode::handle(uint8_t address)
{
    uint8_t sensor = (address >> 4);
    uint8_t key    = (address & 0x0f);

    static uint8_t page = 1;
    uint8_t max_pages = 1;

    clear_screen();
    print_word(0, HOTT_NAME, false);

    // pagination
    char sign[2];
    sign[1] = 0;
    if (page < max_pages) {
        sign[0] = '>';
    } else {
        sign[0] = ' ';
    }
    print_word(20, sign, false);

    sign[0] = '<';
    print_word(19, sign, false);

    // page counter
    char page_buffer[3];
    itoa(page, page_buffer, 10);
    print_word(15, page_buffer, false);
    print_word(16, "/", false);
    itoa(max_pages, page_buffer, 10);
    print_word(17, page_buffer, false);

    switch (key) {
    case TEXT_MODE_KEY_ESC:
        if (page > 0) {
            page--;
        }
        break;


    }

    if (page < 1) {
        _msg.fill1 = 0x01;
        page = 1;
    } else {
        _msg.fill1 = sensor;
    }
}

void AP_HoTT_TextMode::clear_screen()
{
    // fill with spaces (ascii code 32)
    memset(_msg.msg_txt, 0x20, sizeof(_msg.msg_txt));
}

void AP_HoTT_TextMode::print_word(uint8_t pos, char const *text, bool inverted)
{
    for (uint8_t index = 0; ; index++) {
        if (text[index] == 0x0) {
            break;
        } else {
            _msg.msg_txt[pos + index] = text[index] + (inverted ? 128 : 0);
        }
    }
}