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
   HoTT Telemetry library
*/
#include "AP_HoTT_Telem.h"
extern const AP_HAL::HAL& hal;

#define NUM_MODES   19
const char hott_flight_mode_strings[NUM_MODES+1][10] = {
    "STABILIZE",    // 0
    "ACRO",         // 1
    "ALT_HOLD",     // 2
    "AUTO",         // 3
    "GUIDED",       // 4
    "LOITER",       // 5
    "RTL",          // 6
    "CIRCLE",       // 7
    "???",          // 8
    "LAND",         // 9
    "OF_LOITER",    // 10
    "DRIFT",        // 11
    "???",          // 12
    "SPORT",        // 13
    "FLIP",         // 14
    "AUTOTUNE",     // 15
    "POSHOLD",      // 16
    "BRAKE",        // 17
    "THROW",        // 18
    "???"
};

//constructor
AP_HoTT_Telem::AP_HoTT_Telem(AP_AHRS &ahrs, AP_BattMonitor &battery, Location &current_loc, Compass &compass) :
    _ahrs(ahrs),
    _battery(battery),
    _current_loc(current_loc),
    _compass(compass),
    _port(NULL),
    _initialised_uart(false),
    _mode(0),
    _last_delay_ms(0),
    _wp_distance(0),
    _wp_bearing(0),
    _home_distance(0),
    _home_bearing(0),
    _armed(false),
    _hott_status(HottIdle),
    _last_delay_1s(0),
    _climbrate1s(0),
    _climbrate3s(0),
    _climbrate10s(0),
    _electric_time(0)
    {}
    
// init - perform require initialisation including detecting which protocol to use
void AP_HoTT_Telem::init(const AP_SerialManager& serial_manager)
{
    // check for HoTT Port
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_HoTT, 0))) {
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_HoTT_Telem::hott_tick, void));

        // Init GPS Module Message
        memset(&_hott_gps_msg, 0, sizeof(struct HOTT_GPS_MSG));
        _hott_gps_msg.start_byte    = BINARY_MODE_START_BYTE;
        _hott_gps_msg.gps_sensor_id = GPS_SENSOR_ID;
        _hott_gps_msg.sensor_id     = GPS_SENSOR_TEXT_ID;
        _hott_gps_msg.version       = GPS_SENSOR_TYPE_GRAUPNER;
        _hott_gps_msg.stop_byte     = BINARY_MODE_STOP_BYTE;
        
        // Init EAM Module Message
        memset(&_hott_eam_msg, 0, sizeof(struct HOTT_EAM_MSG));
        _hott_eam_msg.start_byte    = BINARY_MODE_START_BYTE;
        _hott_eam_msg.eam_sensor_id = EAM_SENSOR_ID;
        _hott_eam_msg.sensor_id     = EAM_SENSOR_TEXT_ID;
        _hott_eam_msg.stop_byte     = BINARY_MODE_STOP_BYTE;

        // Init Vario Module Message
        memset(&_hott_vario_msg, 0, sizeof(struct HOTT_VARIO_MSG));
        _hott_vario_msg.start_byte      = BINARY_MODE_START_BYTE;
        _hott_vario_msg.vario_sensor_id = VARIO_SENSOR_ID;
        _hott_vario_msg.sensor_id       = VARIO_SENSOR_TEXT_ID;
        _hott_vario_msg.stop_byte       = BINARY_MODE_STOP_BYTE;
    }
    
    if (_port != NULL) {
        // we don't want flow control
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }    
}

void AP_HoTT_Telem::update_data(uint8_t control_mode, uint32_t wp_distance, int32_t wp_bearing, int32_t home_distance, int32_t home_bearing, bool armed)
{
    // return immediately if not initialised
    if (!_initialised_uart) {
        return;
    }
    
    _mode = control_mode;
    if (_mode > NUM_MODES) {
    	  _mode = NUM_MODES;
    }
    
    _wp_distance = wp_distance;
    _wp_bearing = wp_bearing;
    _home_distance = home_distance;
    _home_bearing = home_bearing;
    _armed = armed;
    
    update_gps_data();
    update_eam_data();
    update_vario_data();
    
    // Things to be done every 1 sec
    if (AP_HAL::millis() - _last_delay_1s > 1000) {
    	  _last_delay_1s = AP_HAL::millis();
    	
    	  // update vario data
    	  if (_current_loc.flags.relative_alt) {
            processClimbrate(_current_loc.alt); 
        } else {
            processClimbrate(_current_loc.alt - _ahrs.get_home().alt);
        }
        
        // update electrical time
        if(_armed)
            _electric_time++;
    	  
    }
}

/*
  init_uart - initialise uart
  this must be called from hott_tick which is called from the 1khz scheduler
  because the UART begin must be called from the same thread as it is used from
 */
void AP_HoTT_Telem::init_uart()
{
    // initialise uart
    _port->begin(AP_SERIALMANAGER_HOTT_BAUD, AP_SERIALMANAGER_HOTT_BUFSIZE_RX, AP_SERIALMANAGER_HOTT_BUFSIZE_TX);
    _initialised_uart = true;
}

/*
  hott_tick - main call to send updates to transmitter
  called by scheduler at a high rate
*/
void AP_HoTT_Telem::hott_tick(void)
{
    // check UART has been initialised
    if (!_initialised_uart) {
        init_uart();
    }
    
    // check if there is any data to send
    switch(_hott_status) {
        case HottSendGPS:
            send_data((uint8_t*)&_hott_gps_msg);
        	  break;
        	  
        case HottSendEAM:
        	  send_data((uint8_t*)&_hott_eam_msg);
        	  break;

        case HottSendVario:
        	  send_data((uint8_t*)&_hott_vario_msg);
        	  break;
        	
        default:
            break;
    }
    
    // ignore any new requests until the entire data frame is sent
    if (_hott_status > HottRcvId) {
        return;
    }
    
    int16_t numc;
    numc = _port->available();

    // check if available is negative
    if (numc < 0) {
        return;
    }

    for (int16_t i = 0; i < numc; i++) {
        int16_t readbyte = _port->read();
        if (_hott_status == HottIdle) {
    	      if (readbyte == BINARY_MODE_REQUEST_ID) {
    	          _hott_status = HottRcvMode;
    	      }
    	  } else {
    	  	  _current_msg_pos = 0;
    	  	  _current_delay_ms = POST_READ_DELAY_IN_MS;
    	  	  _checksum = 0;

    	      switch (readbyte) {
    	  	      case GPS_SENSOR_ID:
    	  	          _hott_status = HottSendGPS;
    	  	          _current_msg_size = sizeof(struct HOTT_GPS_MSG);
    	  	      	  break;
    	  	      	  
    	  	      case EAM_SENSOR_ID:
    	  	          _hott_status = HottSendEAM;
    	  	          _current_msg_size = sizeof(struct HOTT_EAM_MSG);
    	  	          break;
    	  	          
    	  	      case VARIO_SENSOR_ID:
    	  	          _hott_status = HottSendVario;
    	  	          _current_msg_size = sizeof(struct HOTT_VARIO_MSG);
    	  	          break;

    	  	      default:
    	  	      	  _hott_status = HottIdle;
    	  	      	  break;
    	      }
    	  }
    }
}

void AP_HoTT_Telem::send_data(uint8_t *buffer)
{
    uint32_t now = AP_HAL::millis();
    if (now - _last_delay_ms > _current_delay_ms) {
        _last_delay_ms = AP_HAL::millis();
        	  	  
        if (_current_msg_pos < _current_msg_size) {
            if (_current_msg_pos == _current_msg_size - 1) {
                /* Set the checksum: the first uint8_t is taken as the checksum. */
                buffer[_current_msg_pos] = _checksum & 0xff;
            } else {
                _checksum += buffer[_current_msg_pos];
            }
            _port->write(&buffer[_current_msg_pos], sizeof(buffer[_current_msg_pos]));
                    
            _current_delay_ms = POST_WRITE_DELAY_IN_MS;
            _current_msg_pos++;
        } else {
            _hott_status = HottIdle;
        }
    }
}

void AP_HoTT_Telem::convertLatLong(float degree, uint8_t &posNS_EW, uint16_t &degMinutes, uint16_t &degSeconds) {
    degree = degree / 10000000.0f;
    if (degree >= 0) {
        posNS_EW = 0;
    } else {
        posNS_EW = 1;
        degree = -degree;
    }
    
    int16_t deg = int (degree) ;
    float mmmm = 60 * ( degree - deg );
    int16_t minu = (int( mmmm ));
    mmmm -= minu;
    degSeconds = mmmm * 1E4;
    degMinutes = (deg * 100) + minu;
}

//
// calculates and maintans climbrate changes
// called every 1s with current altitude as input
//
void AP_HoTT_Telem::processClimbrate(int16_t currentAltitude) {
    static int16_t altitudeData[ALTITUDE_HISTORY_DATA_COUNT];   //all values in cm
    static int8_t nextData = -1;
    static bool gotAllDatapoints = false;
    
    //clear data?
    if(nextData == -1) {
        for(int i = 0; i < ALTITUDE_HISTORY_DATA_COUNT; i++)
            altitudeData[i] = 0;
        nextData = 0;
    }
    
    // save next altitude data
    int8_t x = nextData;
    altitudeData[nextData++] = currentAltitude;
    if(nextData == ALTITUDE_HISTORY_DATA_COUNT) {
        nextData = 0;
        if(!gotAllDatapoints) {
            gotAllDatapoints = true;    //ready for calculations
        }
    }
    if(!gotAllDatapoints)   //wait for all data points requred for calculation
        return;
    
    int8_t y = (x - 1 < 0) ? ALTITUDE_HISTORY_DATA_COUNT - 1 : x - 1;
    _climbrate1s = altitudeData[x] - altitudeData[y];

    y = (x - 2 < 0) ? ALTITUDE_HISTORY_DATA_COUNT + (x - 2) : x - 2;
    _climbrate3s = altitudeData[x] - altitudeData[y];

#if ALTITUDE_HISTORY_DATA_COUNT != 10
#error "ALTITUDE_HISTORY_DATA_COUNT has to be 10... or adapt the code below!"
#endif
    _climbrate10s = altitudeData[x] - altitudeData[nextData];
}

void AP_HoTT_Telem::update_gps_data()
{
	  const AP_GPS &gps = _ahrs.get_gps();

    // Altitude
    if (_current_loc.flags.relative_alt) {
    	  // Mean sea level altitude
        (uint16_t &)_hott_gps_msg.msl_altitude_L = (_current_loc.alt + _ahrs.get_home().alt) / 100;
        
         // Meters above ground
        (uint16_t &)_hott_gps_msg.altitude_L = 500 + (_current_loc.alt / 100);
    } else {
    	  // Mean sea level altitude
        (uint16_t &)_hott_gps_msg.msl_altitude_L = _current_loc.alt / 100;
        
        // Meters above ground
        (uint16_t &)_hott_gps_msg.altitude_L = 500 + ((_current_loc.alt - _ahrs.get_home().alt) / 100);
    }
    
    // Flight Direction
    _hott_gps_msg.flight_direction = gps.ground_course_cd() / 200; // in 2* steps
    
    // Ground Speed
    (uint16_t &)_hott_gps_msg.gps_speed_L = (uint16_t)((float)((gps.ground_speed()) * 3.6));
    
    // GPS Status
    switch(gps.status()) {
        case AP_GPS::GPS_OK_FIX_3D:
            _hott_gps_msg.alarm_invers2 = 0;
            _hott_gps_msg.gps_fix_char  = '3';  
            _hott_gps_msg.free_char3    = '3';  //3D Fix according to specs...
    		    break;

    	  case AP_GPS::GPS_OK_FIX_2D:
    		    //No GPS Fix
	          _hott_gps_msg.alarm_invers2 = 1;
	          _hott_gps_msg.gps_fix_char  = '2';
	          _hott_gps_msg.free_char3    = '2';
	          (uint16_t &)_hott_gps_msg.home_distance_L = 0; // set distance to 0 since there is no GPS signal
         	  break;

    	  default:
	    	    //No GPS Fix
	          _hott_gps_msg.alarm_invers2 = 1;
	          _hott_gps_msg.gps_fix_char  = '-';
	          _hott_gps_msg.free_char3    = '-';
	          (uint16_t &)_hott_gps_msg.home_distance_L = 0; // set distance to 0 since there is no GPS signal
    }
    
    // Home distance
    switch(_mode) {
        case AUTO:
        case LOITER:
        	  //Use home direction field to display direction an distance to next waypoint
        	  (uint16_t &)_hott_gps_msg.home_distance_L = _wp_distance / 100;
        	  _hott_gps_msg.home_direction = _wp_bearing / 200;
        	  _hott_gps_msg.free_char1 = 'W';
            _hott_gps_msg.free_char2 = 'P';
        	  break;
        
        default:
        	  //Display Home direction and distance
        	  (uint16_t &)_hott_gps_msg.home_distance_L = _home_distance / 100;
            _hott_gps_msg.home_direction = _home_bearing / 200;
        	  _hott_gps_msg.free_char1 = 'H';
            _hott_gps_msg.free_char2 = 'O';
            break;
    }
    
    // Coordinates
    convertLatLong(gps.location().lat, (uint8_t &)_hott_gps_msg.pos_NS, (uint16_t &)_hott_gps_msg.pos_NS_dm_L, (uint16_t &)_hott_gps_msg.pos_NS_sec_L);
    convertLatLong(gps.location().lng, (uint8_t &)_hott_gps_msg.pos_EW, (uint16_t &)_hott_gps_msg.pos_EW_dm_L, (uint16_t &)_hott_gps_msg.pos_EW_sec_L);
    
    // Satelite Count
    _hott_gps_msg.gps_satelites = gps.num_sats();
    
    // Compass
    _hott_gps_msg.angle_compass = ToDeg(_compass.calculate_heading(_ahrs.get_rotation_body_to_ned())) / 2;
    
    // Roll/Nick Angle
    _hott_gps_msg.angle_roll = _ahrs.roll_sensor / 200;
    _hott_gps_msg.angle_nick = _ahrs.pitch_sensor / 200;
    
    // Climbrate
    (int16_t &)_hott_gps_msg.climbrate_L = 30000 + _climbrate1s;  
    _hott_gps_msg.climbrate3s            = 120   + (_climbrate3s / 100);  // 0 m/3s

    // GPS Time
    uint32_t t = gps.time_week_ms() % (60 * 60 * 24 * 7);
    _hott_gps_msg.gps_time_h = t / 3600000;
    t -= (_hott_gps_msg.gps_time_h * 3600000);
  
    _hott_gps_msg.gps_time_m = t / 60000;
    t -= _hott_gps_msg.gps_time_m * 60000;
  
    _hott_gps_msg.gps_time_s = t / 1000;
    _hott_gps_msg.gps_time_sss = t - (_hott_gps_msg.gps_time_s * 1000);
}

void AP_HoTT_Telem::update_eam_data()
{
	  const AP_GPS &gps = _ahrs.get_gps();
	  
    // Battery
    (uint16_t &)_hott_eam_msg.main_voltage_L = (uint16_t)(_battery.voltage() * (float)10.0);
    (uint16_t &)_hott_eam_msg.current_L      = (uint16_t)(_battery.current_amps() * (float)10.0);
    (uint16_t &)_hott_eam_msg.batt_cap_L     = (uint16_t)(_battery.current_total_mah() / (float)10.0);
    
    // Climbrate
    _hott_eam_msg.climbrate3s             = 120   + (_climbrate3s / 100);  // 0 m/3s using filtered data here
    (uint16_t &)_hott_eam_msg.climbrate_L = 30000 + _climbrate1s;
    
    // Electric time. Time the APM is ARMED
    _hott_eam_msg.electric_min = _electric_time / 60;
    _hott_eam_msg.electric_sec = _electric_time % 60;

    // Ground Speed
    (uint16_t &)_hott_eam_msg.speed_L = ((float)((gps.ground_speed()) * 3.6));
}

void AP_HoTT_Telem::update_vario_data()
{
	  const uint8_t ARMED_STR[]   = "ARMED";
    const uint8_t DISARMED_STR[] = "DISARMED";
    
    static int16_t max_altitude = 0;
    static int16_t min_altitude = 0;

    // Altitude
    if (_current_loc.flags.relative_alt) {
        (uint16_t &)_hott_vario_msg.altitude_L = 500 + (_current_loc.alt / 100);
    } else {
        (uint16_t &)_hott_vario_msg.altitude_L = 500 + ((_current_loc.alt - _ahrs.get_home().alt) / 100);
    }
    
    // Altitude Max
    if(_hott_vario_msg.altitude_L > max_altitude && _armed) //calc only in ARMED mode
        max_altitude = _hott_vario_msg.altitude_L;
    (int16_t &)_hott_vario_msg.altitude_max_L = 500 + (max_altitude / 100);
    
    // Altitude Min
    if(_hott_vario_msg.altitude_L < min_altitude && _armed) //calc only in ARMED mode
        min_altitude = _hott_vario_msg.altitude_L;
    (int16_t &)_hott_vario_msg.altitude_min_L = 500 + (min_altitude / 100);
    
    // Climbrate
    (int16_t &)_hott_vario_msg.climbrate_L    = 30000 + _climbrate1s;
    (int16_t &)_hott_vario_msg.climbrate3s_L  = 30000 + _climbrate3s;
    (int16_t &)_hott_vario_msg.climbrate10s_L = 30000 + _climbrate10s;

    // Compass
    _hott_vario_msg.compass_direction = ToDeg(_compass.calculate_heading(_ahrs.get_rotation_body_to_ned())) / 2;
    
    // Armed Text
    char *pArmedStr = (char *)DISARMED_STR;
    if (_armed) {
        pArmedStr = (char *)ARMED_STR;
    }
	
	  // Clear line
    memset(_hott_vario_msg.text_msg, 0x20, VARIO_MSG_TEXT_LEN);
    
    // Armed Mode
    uint8_t len = strlen(pArmedStr);
    memcpy((uint8_t*)_hott_vario_msg.text_msg, pArmedStr, len);
    
	  // Flight Mode
	  memcpy((uint8_t*)&_hott_vario_msg.text_msg[len + 1], hott_flight_mode_strings[_mode], strlen(hott_flight_mode_strings[_mode]));
}