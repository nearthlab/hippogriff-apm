#include "AP_GPS.h"
#include "AP_GPS_NOVATEL.h"

#include <DataFlash/DataFlash.h>

#define NVOATEL_DEBUGGING 0

#define STAT_FIX_VALID 0x01

extern const AP_HAL::HAL& hal;

#if NVOATEL_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

AP_GPS_NOVATEL::AP_GPS_NOVATEL(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    _step(0),
    _msg_id(0),
    _payload_length(0),
    _payload_counter(0),
    _fix_count(0),
    _new_position(0),
    _new_speed(0),
    next_fix(AP_GPS::NO_FIX)
{
}

// Process bytes available from the stream
//
// The stream is assumed to contain only messages we recognise.  If it
// contains other messages, and those messages contain the preamble
// bytes, it is possible for this code to fail to synchronise to the
// stream immediately.  Without buffering the entire message and
// re-processing it from the top, this is unavoidable. The parser
// attempts to avoid this when possible.
//
bool
AP_GPS_NOVATEL::read(void)
{
    uint8_t data;
    int16_t numc;
    bool parsed = false;
    int32_t ck_idx;

    numc = port->available();
    for (int16_t i = 0; i < numc; i++) {        // Process bytes received

        // read the next byte
        data = port->read();
        reset:
        switch(_step) {


        case STATE_PREAMBLE1:
        	_header_counter = 0;
			if(data == PREAMBLE1)
			{
				_packet.hdr.bytes[_header_counter++] = data;
				_step++;
			}
			else
				_step = STATE_PREAMBLE1;
			break;

        case STATE_PREAMBLE2:
			if(data == PREAMBLE2)
			{
				_packet.hdr.bytes[_header_counter++] = data;
				_step++;
			}
			else
				_step = STATE_PREAMBLE1;
			break;

        case STATE_PREAMBLE3:
        	if(data == PREAMBLE3)
        	{
        		_packet.hdr.bytes[_header_counter++] = data;
        		_step++;
        	}
			else
				_step = STATE_PREAMBLE1;
			break;

        case STATE_GET_HEADER:
        	if (_header_counter < sizeof(novatel_header)) {
        		_packet.hdr.bytes[_header_counter++] = data;
        		if(_header_counter == sizeof(novatel_header))
				{
        			_msg_id = _packet.hdr.data.message_id;
        			_payload_length =  _packet.hdr.data.message_length;
        			_payload_counter = 0;
					_step++;
				}
        	}
        	else
        	{
        		// Wrong case, reset!
        		_step = STATE_PREAMBLE1;
        	}
			break;

        case STATE_GET_DATA:
			if (_payload_counter < sizeof(_packet.msg)) {
				_packet.msg.bytes[_payload_counter] = data;
			}
			if (++_payload_counter == _payload_length)
			{
				_step++;
				ck_idx = 0;
			}
        	break;

        case STATE_CHECKSUM1:
		case STATE_CHECKSUM2:
		case STATE_CHECKSUM3:
			ck_idx = _step - STATE_CHECKSUM1;
			_ck[ck_idx] = data;
			_step++;
			break;

        case STATE_CHECKSUM4:
        	_ck[3] = data;
        	if(_check_checksum())
        	{
        		// Parse Ok
        		parsed = true;
        		_parse_gps();
        	}
        	_step = STATE_PREAMBLE1;
			break;
        }
    }



    return parsed;
}

bool
AP_GPS_NOVATEL::_check_checksum(void)
{
	uint32_t calculated;
	uint32_t actual;

	calculated = _calculate_block_crc32(sizeof(novatel_header) + _payload_length, (uint8_t*)&_packet);
	memcpy(&actual, _ck, sizeof(uint32_t));

	return (calculated == actual);
}

#define CRC32_POLYNOMIAL 0xEDB88320L
uint32_t
AP_GPS_NOVATEL::_calculate_crc32(int32_t i)
{
	uint32_t ulCRC;
	int j;
	ulCRC = i;
	for ( j = 8 ; j > 0; j-- )
	{
		if ( ulCRC & 1 )
			ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return ulCRC;
}

uint32_t
AP_GPS_NOVATEL::_calculate_block_crc32(
		uint32_t ulCount,	/* Number of bytes in the data block */
		uint8_t *ucBuffer)	/* Data block */
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while ( ulCount-- != 0 )
	{
		ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
		ulTemp2 = _calculate_crc32( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return( ulCRC );
}

AP_GPS::GPS_Status
AP_GPS_NOVATEL::_convert_gps_status(uint32_t pos_type)
{
	switch(pos_type)
	{
	case FIX_SINGLE:
		return AP_GPS::GPS_OK_FIX_3D;

	case FIX_PSRDIFF:
	case FIX_L1_FLOAT:
	case FIX_IONOFREE_FLOAT:
	case FIX_NARROW_FLOAT:
		return AP_GPS::GPS_OK_FIX_3D_DGPS;

	case FIX_L1_INT:
	case FIX_NARROW_INT:
		return AP_GPS::GPS_OK_FIX_3D_RTK;

	case FIX_NONE:
	default:
		return AP_GPS::NO_FIX;
	}

	return AP_GPS::NO_FIX;
}

void
AP_GPS_NOVATEL::_fill_ned_vel(double hor_spd, double trk_gnd, double vert_spd)
{
	double vel_n = hor_spd*cos(radians(trk_gnd));
	double vel_e = hor_spd*sin(radians(trk_gnd));

	state.velocity.x = vel_n * 0.01f;
	state.velocity.y = vel_e * 0.01f;
	state.velocity.z = -vert_spd * 0.01f;
}

bool
AP_GPS_NOVATEL::_parse_gps(void)
{
	novatel_header* p_hdr = &(_packet.hdr.data);
	novatel_bestposb* p_pos = &(_packet.msg.bestposb);
	novatel_bestvelb* p_vel = &(_packet.msg.bestvelb);

    switch (_msg_id) {
    case MSG_BESTPOSB:
        _last_pos_time        = p_hdr->seconds;
        state.location.lng    = (int32_t)(p_pos);
        state.location.lat    = (int32_t)(p_pos->lat * 1e7);
        state.location.alt    = (int32_t)(p_pos->hgt * 1e2);
        state.status          = _convert_gps_status(p_pos->pos_type);
        state.num_sats		  = (uint8_t)p_pos->n_sol_sv;
        _new_position = true;
        state.horizontal_accuracy = p_pos->lat_sigma * 1.0e-3f;
        state.vertical_accuracy = p_pos->hgt_sigma * 1.0e-3f;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;

        if (state.status >= AP_GPS::GPS_OK_FIX_3D) {
			state.last_gps_time_ms = hal.scheduler->millis();
			state.time_week_ms    = p_hdr->seconds;
            state.time_week       = p_hdr->week;
        }
        break;

    case MSG_BESTVELB:
        _last_vel_time         = p_hdr->seconds;
        state.ground_speed     = (float)p_vel->hor_spd * 0.01f;        // m/s
        // Heading 2D deg * 100000 rescaled to deg * 100
        state.ground_course_cd = wrap_360_cd(p_vel->trk_gnd / 1000);
        state.have_vertical_velocity = true;
        _fill_ned_vel(
			p_vel->hor_spd,
			p_vel->trk_gnd,
			p_vel->vert_spd
			);
        state.have_speed_accuracy = false;
        _new_speed = true;
        break;

    default:
        Debug("Unexpected message 0x%02x", (unsigned)_msg_id);
        return false;
    }
    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed) {
        _new_speed = _new_position = false;
        _fix_count++;
        return true;
    }
    return false;
}

void
AP_GPS_NOVATEL::inject_data(uint8_t *data, uint8_t len)
{

    if (port->txspace() > len) {
        port->write(data, len);
    } else {
        Debug("ERB: Not enough TXSPACE");
    }
}

/*
  detect a ERB GPS. Adds one byte, and returns true if the stream
  matches a ERB
 */
bool
AP_GPS_NOVATEL::_detect(struct NOVATEL_detect_state &state, uint8_t data)
{
    return true;
}
