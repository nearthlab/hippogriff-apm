#include "AP_GPS.h"
#include "AP_GPS_TRIMBLE.h"

#include <DataFlash/DataFlash.h>

#define TRIMBLE_DEBUGGING 0

#define STAT_FIX_VALID 0x01

extern const AP_HAL::HAL& hal;

#if NVOATEL_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

AP_GPS_TRIMBLE::AP_GPS_TRIMBLE(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    _step(0),
    _msg_id(0),
    _payload_length(0),
    _payload_counter(0),
    _fix_count(0),
    _new_position(0),
    _new_speed(0),
    _ck(0),
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
AP_GPS_TRIMBLE::read(void)
{
	uint8_t data;
	int16_t numc;
	bool parsed = false;

	numc = port->available();
	for (int16_t i = 0; i < numc; i++) {        // Process bytes received

		// read the next byte
		data = port->read();
		//reset:
		switch (_step) {


		case STATE_PREAMBLE:
			_header_counter = 0;
			if (data == GSOF_STX)
			{
				_packet.hdr.bytes[_header_counter++] = data;
				_step++;
			}
			else
				_step = STATE_PREAMBLE;
			break;

		case STATE_GET_HEADER:
			if (_header_counter < sizeof(trimble_header)) {
				_packet.hdr.bytes[_header_counter++] = data;
				if (_header_counter == sizeof(trimble_header))
				{
					_payload_length = _packet.hdr.data.length;
					if (_payload_length != sizeof(trimble_message))
					{
						_step = STATE_PREAMBLE;
						break;
					}
					_payload_counter = 0;
					_step++;
				}
			}
			else
			{
				// Wrong case, reset!
				_step = STATE_PREAMBLE;
			}
			break;

		case STATE_GET_DATA:
			if (_payload_counter < sizeof(_packet.msg_buf)) {
				_packet.msg_buf[_payload_counter] = data;
			}
			if (++_payload_counter == _payload_length)
			{
				_step++;
			}
			break;

		case STATE_CHECKSUM:
			_ck = data;
			if (_check_checksum())
			{
				_step++;
			}
			else
			{
				_step = STATE_PREAMBLE;
			}
			break;

		case STATE_POSTAMBLE:
			if (data == GSOF_ETX)
			{
				// Parse Ok
				parsed = true;
				_parse_trimble();
			}
			_step = STATE_PREAMBLE;
			break;
		default:
			PX4_INFO("Step Error!");
			_step = STATE_PREAMBLE;
			break;
		}

	}

	return parsed;
}

bool
AP_GPS_TRIMBLE::_check_checksum(void)
{
	uint8_t calculated = 0;
	int i;

	uint8_t* data = _packet.msg_buf;
	int32_t len = _packet.hdr.data.length;

	calculated += _packet.hdr.data.status;
	calculated += _packet.hdr.data.packet_type;
	calculated += len;

	if (len > (int32_t)sizeof(_packet.msg_buf))
	{
		PX4_INFO("Out of buf %d, %d", len, sizeof(_packet.msg_buf));
		return false;
	}

	for (i = 0; i < len; i++)
	{
		calculated += data[i];
	}

	return (calculated == _ck);
}

AP_GPS::GPS_Status
AP_GPS_TRIMBLE::_convert_gps_status(int16_t position_flag)
{
	trimble_position_flag flag;

	memcpy(&flag, &position_flag, sizeof(trimble_position_flag));

	if ((flag.reserved1 != 1) || (flag.reserved2 != 0))
	{
		PX4_INFO("Flag Eror");
		return AP_GPS::NO_FIX;
	}

	if ((flag.height_fix != 1) || (flag.horizontal_fix != 1))
		return AP_GPS::NO_FIX;

	if (flag.diff_pos != 1)
		return AP_GPS::GPS_OK_FIX_3D;

	if (flag.diff_pos_method2 != 1)
		return AP_GPS::GPS_OK_FIX_3D_DGPS;

	return AP_GPS::GPS_OK_FIX_3D_RTK;
}

bool
AP_GPS_TRIMBLE::_is_vel_ok(trimble_velocity* p_vel)
{
	//if ((p_vel->velocity_flags.validity == 1) && (p_vel->velocity_flags.computation == 0))
	if ((p_vel->velocity_flags.validity == 1))
		return true;
	else
		return false;
}

void
AP_GPS_TRIMBLE::_fill_ned_vel(double hor_spd, double heading, double vert_spd)
{
	double vel_n = hor_spd*cos(heading);
	double vel_e = hor_spd*sin(heading);

	state.velocity.x = vel_n;
	state.velocity.y = vel_e;
	state.velocity.z = -vert_spd;
}

bool
AP_GPS_TRIMBLE::_read_trimble_record(void)
{
	int32_t idx = 0;
	int32_t len = 0;
	int32_t type;
	int32_t total_length = _packet.hdr.data.length;
	uint8_t* buf = _packet.msg_buf;

	_packet.msg.transmission_number = buf[idx++];
	_packet.msg.page_index = buf[idx++];
	_packet.msg.max_page_index = buf[idx++];

	if (total_length != sizeof(trimble_message))
	{
		PX4_INFO("Wrong total length total%d, sizemsg %d", total_length, sizeof(trimble_message));
		return false;
	}

	while (idx < total_length)
	{
		type = buf[idx + 0];
		len = buf[idx + 1] + 2;	// type + len + datalength

		if ((idx + len) > (int32_t)sizeof(_packet.msg_buf))
		{
			PX4_INFO("Exceeds buf idx:%d, len%d, sizebuf %d", idx, len, sizeof(_packet.msg_buf));
			return false;
		}

		switch (type) {

		case GSOF_TIME:
			if (len != sizeof(trimble_time))
			{
				PX4_INFO("Wrong Record Len %d %d", type, len);
				return false;
			}
			memcpy(&(_packet.msg.time), &(buf[idx]), sizeof(trimble_time));
			break;

		case GSOF_LLH:
			if (len != sizeof(trimble_llh))
			{
				PX4_INFO("Wrong Record Len %d %d", type, len);
				return false;
			}
			memcpy(&(_packet.msg.llh), &(buf[idx]), sizeof(trimble_llh));
			break;

		case GSOF_VELOCITY:
			if (len != sizeof(trimble_velocity))
			{
				PX4_INFO("Wrong Record Len %d %d", type, len);
				return false;
			}
			memcpy(&(_packet.msg.velocity), &(buf[idx]), sizeof(trimble_velocity));
			break;



		default:
			PX4_INFO("Wrong Type %d %d", type, len);
			return false;
		}

		idx += len;
		if (len == 0)
		{
			PX4_INFO("Wrong Length %d", len);
			return false;
		}

	}

	return true;
}

bool
AP_GPS_TRIMBLE::_parse_trimble(void)
{
	trimble_time* p_time = &(_packet.msg.time);
	trimble_llh* p_llh = &(_packet.msg.llh);
	trimble_velocity* p_vel = &(_packet.msg.velocity);

	int32_t gps_time;

	if (!_read_trimble_record())
	{
		PX4_INFO("Trimble Record Error");
		state.status = AP_GPS::NO_FIX;
		return false;
	}

	// Time
	gps_time = cvt_int(p_time->gps_time);
	state.num_sats = (uint8_t)p_time->n_sv;
	state.status = _convert_gps_status(p_time->position_flag);

	if (state.status >= AP_GPS::GPS_OK_FIX_3D) {
		//state.last_gps_time_ms = hal.scheduler->millis();
		state.time_week_ms = gps_time;
		state.time_week = cvt_int(p_time->gps_week_number);
	}

	// LLH
	_last_pos_time = gps_time;
	state.location.lng = (int32_t)(cvt_double(p_llh->longitude) / M_PI * 180.0 * 1.0e7);
	state.location.lat = (int32_t)(cvt_double(p_llh->latitude) / M_PI * 180.0 * 1.0e7);
	state.location.alt = (int32_t)(cvt_double(p_llh->height) * 1e2);

	_new_position = true;
	state.have_horizontal_accuracy = false;
	state.have_vertical_accuracy = false;

	// Velocity
	if (_is_vel_ok(p_vel))
	{
		_last_vel_time = gps_time;
		state.ground_speed = cvt_float(p_vel->speed);        // m/s
		state.ground_course_cd = (int32_t)(cvt_float(p_vel->heading) * 100.0f);

		//PX4_INFO("VEL: %u, %u", p_vel->velocity_flags.validity, p_vel->velocity_flags.computation);
		_fill_ned_vel(
			cvt_float(p_vel->speed),
			cvt_float(p_vel->heading),
			cvt_float(p_vel->vertical_velocity)
		);

		state.have_vertical_velocity = true;
		state.have_speed_accuracy = false;
		_new_speed = true;
	}
	else
	{
		_new_speed = false;
	}

	//PX4_INFO("%d %d %d %d %d %f %f %f", state.status, state.time_week_ms,
		//state.location.lat, state.location.lng, state.location.alt,
		//state.velocity.x, state.velocity.y, state.velocity.z);

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
AP_GPS_TRIMBLE::inject_data(uint8_t *data, uint8_t len)
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
AP_GPS_TRIMBLE::_detect(struct NOVATEL_detect_state &state, uint8_t data)
{
    return true;
}


int32_t
AP_GPS_TRIMBLE::cvt_int(int32_t Src)
{
	int32_t output;
	unsigned char buff1;
	unsigned char *dst, *src;
	dst = (unsigned char*)&output;
	src = (unsigned char*)&Src;

	buff1 = src[0];
	dst[0] = src[3];
	dst[3] = buff1;

	buff1 = src[1];
	dst[1] = src[2];
	dst[2] = buff1;

	return output;
}

float
AP_GPS_TRIMBLE::cvt_float(float Src)
{
	float output;
	unsigned char buff1;
	unsigned char *dst, *src;
	dst = (unsigned char*)&output;
	src = (unsigned char*)&Src;

	buff1 = src[0];
	dst[0] = src[3];
	dst[3] = buff1;

	buff1 = src[1];
	dst[1] = src[2];
	dst[2] = buff1;

	return output;
}

double
AP_GPS_TRIMBLE::cvt_double(double Src)
{
	double output;
	unsigned char buff1;
	unsigned char *dst, *src;
	dst = (unsigned char*)&output;
	src = (unsigned char*)&Src;

	buff1 = src[0];
	dst[0] = src[7];
	dst[7] = buff1;

	buff1 = src[1];
	dst[1] = src[6];
	dst[6] = buff1;

	buff1 = src[2];
	dst[2] = src[5];
	dst[5] = buff1;

	buff1 = src[3];
	dst[3] = src[4];
	dst[4] = buff1;

	return output;
}
