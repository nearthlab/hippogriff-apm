#include "AP_GPS.h"
#include "AP_GPS_TRIMBLE.h"
#include <AP_Math/AP_Math.h>

#include <DataFlash/DataFlash.h>

#define ATOM_DEBUGGING 0

extern const AP_HAL::HAL& hal;

#if ATOM_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

AP_GPS_ATOM::AP_GPS_ATOM(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    _step(STATE_PREAMBLE),
	_counter(0)
{
	_request();
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
AP_GPS_ATOM::read(void)
{
	uint8_t data;
	int16_t numc;
	bool parsed = false;

	numc = port->available();

	for (int16_t i = 0; i < numc; i++) {
		data = port->read();
		parsed |= _parse(data);
	}

	return parsed;
}

/** Calculate Qualcomm 24-bit Cyclical Redundancy Check (CRC-24Q).
*
* The CRC polynomial used is:
* \f[
*   x^{24} + x^{23} + x^{18} + x^{17} + x^{14} + x^{11} + x^{10} +
*   x^7    + x^6    + x^5    + x^4    + x^3    + x+1
* \f]
* Mask 0x1864CFB, not reversed, not XOR'd
*
* \param buf Array of data to calculate CRC for
* \param len Length of data array
* \param crc Initial CRC value
*
* \return CRC-24Q value
*/
uint32_t
AP_GPS_ATOM::crc24q(const uint8_t *buf, uint32_t len, uint32_t crc)
{
	for (uint32_t i = 0; i < len; i++)
		crc = ((crc << 8) & 0xFFFFFF) ^ crc24qtab[((crc >> 16) ^ buf[i]) & 0xff];
	return crc;
}

/**
* Computes CRC-24Q for left-aligned bit message.
* This function is used for left-aligned bit messages, for example SBAS and
* GPS CNAV.
* GPS message is 300 bits total, but 276 bits without CRC. It takes 34.5
* 8-bit bytes, and when computing CRC the message has to be padded with zero
* bits.
*
* \param[in] crc    Initial CRC value
* \param[in] buf    Pointer to MSB-aligned data.
* \param[in] n_bits Number of bits in the data buffer.
* \param[in] invert Flag to compute inverted CRC.
*
* \return CRC-24Q value
*/
uint32_t
AP_GPS_ATOM::crc24q_bits(uint32_t crc, const uint8_t *buf, uint32_t n_bits, bool invert)
{
	uint16_t acc = 0;
	uint8_t  b = 0;
	uint32_t shift = 8 - n_bits % 8;

	for (uint32_t i = 0; i < n_bits / 8; ++i) {
		acc = (acc << 8) | *buf++;
		if (invert) {
			acc ^= 0xFFu;
		}
		b = (acc >> shift) & 0xFFu;
		crc = ((crc << 8) & 0xFFFFFFu) ^ crc24qtab[((crc >> 16) ^ b) & 0xFFu];
	}
	acc = (acc << 8) | *buf;
	if (invert) {
		acc ^= 0xFFu;
	}
	b = (acc >> shift) & 0xFFu;
	crc = ((crc << 8) & 0xFFFFFFu) ^ crc24qtab[((crc >> 16) ^ b) & 0xFFu];

	return crc;
}

void
AP_GPS_ATOM::_decode_start(uint8_t buf[])
{
	_pvt.start.preamble = buf[0];
	_pvt.start.msg_length = ((0x00000011b & buf[1]) << 8) + buf[2];
}

void
AP_GPS_ATOM::_decode_pvt_header(uint8_t buf[])
{
	_pvt.header.nsats_used = ((0x01 & buf[4]) << 5 ) +  ((0xF8 & buf[5]) >> 3);

}

void
AP_GPS_ATOM::_decode_pvt_coo(uint8_t buf[])
{
	_pvt.coo.block_size = buf[0];
	_pvt.coo.block_id = (0x11110000b & buf[1]) >> 4;
	_pvt.coo.pos_type = (0x00001111b & buf[1]);
	_pvt.coo.gnss_mask = buf[2];
	_pvt.coo.pos_mode = (0x11100000b & buf[3]) >> 5;
	_pvt.coo.pos_smoothing = (0x00011100b & buf[3]) >> 2;
	_pvt.coo.pdop = ((0x3F & buf[4]) << 4) + ((0xF0 & buf[5]) >> 4);
	_pvt.coo.hdop = ((0x0F & buf[5]) << 6) + ((0xFC & buf[6]) >> 2);

	_pvt.coo.x = ((uint64_t)(0x03 & buf[6]) * 68719476736ull)
		+ ((uint64_t)(buf[7]) * 268435456ull)
		+ (uint64_t)((buf[8]) << 20)
		+ (uint64_t)((buf[9]) << 12)
		+ (uint64_t)((buf[10]) << 4)
		+ (uint64_t)((0xFC & buf[11]) >> 4);

	_pvt.coo.y = ((uint64_t)(0x0F & buf[11]) * 17179869184ull)
		+ ((uint64_t)(buf[12]) * 67108864ull)
		+ (uint64_t)((buf[13]) << 18)
		+ (uint64_t)((buf[14]) << 10)
		+ (uint64_t)((buf[15]) << 2)
		+ (uint64_t)((0xC0 & buf[16]) >> 6);

	_pvt.coo.z = ((uint64_t)(0x3F & buf[16]) * 4294967296ull)
		+ ((uint64_t)(buf[17]) * 16777216ull)
		+ (uint64_t)((buf[18]) << 16)
		+ (uint64_t)((buf[19]) << 8)
		+ (uint64_t)((buf[20]));
}

void
AP_GPS_ATOM::_decode_pvt_vel(uint8_t buf[])
{
	_pvt.vel.block_size = buf[0];
	_pvt.vel.block_id = (0x11110000b & buf[1]) >> 4;

	_pvt.vel.vx = ((0x0F & buf[1]) << 21)
		+ ((buf[2]) << 13)
		+ ((buf[3]) << 5)
		+ ((0xF8 & buf[4]) >> 3);

	_pvt.vel.vy = ((0x07 & buf[4]) << 22)
		+ ((buf[5]) << 14)
		+ ((buf[6]) << 6)
		+ ((0x3F & buf[7]) >> 2);

	_pvt.vel.vz = ((0x03 & buf[7]) << 23)
		+ ((buf[8]) << 15)
		+ ((buf[9]) << 7)
		+ ((0x3F & buf[10]) >> 1);
}

void
AP_GPS_ATOM::_decode_crc(uint8_t buf[])
{
	_pvt.crc.crc = (buf[0] << 16) + (buf[1] << 8) + buf[2];
}

bool
AP_GPS_ATOM::_decode()
{
	_convert_pos_ecef_to_llh();
	_calculate_ecef2ned(_pos_llh.x*RAD_TO_DEG, _pos_llh.y*RAD_TO_DEG);
	_convert_vel_ecef_to_ned();

	if(fabs(_pos_llh.x * RAD_TO_DEG) > 90.0f)
		return false;

	state.location.lat = (int32_t)(_pos_llh.x * RAD_TO_DEG * 1e7);
	state.location.lng = (int32_t)(_pos_llh.y * RAD_TO_DEG * 1e7);
	state.location.alt = (int32_t)(_pos_llh.z * RAD_TO_DEG * 1e2);
	state.have_horizontal_accuracy = false;
	state.have_vertical_accuracy = false;

	state.status          = _get_gps_status();
	state.num_sats		  = (uint8_t)_pvt.header.nsats_used;
	if (state.status >= AP_GPS::GPS_OK_FIX_3D) {
		state.last_gps_time_ms = hal.scheduler->millis();
		//state.time_week_ms    = p_hdr->seconds;
		//state.time_week       = p_hdr->week;
	}

	state.velocity.x = _vel_ned.x;
	state.velocity.y = _vel_ned.y;
	state.velocity.z = _vel_ned.z;

	/*
	PX4_INFO("%d %d %d %d %d %.6f %.6f %.6f",
			state.status, state.num_sats,
			state.location.lat,
			state.location.lng,
			state.location.alt,
			_vel_ned.x,
			_vel_ned.y,
			_vel_ned.z);*/

	return true;
}

bool
AP_GPS_ATOM::_parse(uint8_t data)
{
	uint8_t parsed = false;

	if (_step == STATE_PREAMBLE) {
		_counter = 0;
	}

	if (_counter < sizeof(_buf)) {
		_buf[_counter++] = data;
	} else {
		_step = STATE_PREAMBLE;
	}

	switch (_step) {
	case STATE_PREAMBLE:
		if (data == 0xD3) {
			_step++;
		}
		break;

	case STATE_LENGTH:
		if (_counter >= 3) {
			_decode_start(&_buf[0]);
			if (_pvt.start.msg_length == (sizeof(_buf) - 6)) {
				_step++;
			} else {
				_step = STATE_PREAMBLE;
			}
		}
		break;

	case STATE_HEADER:
		if (_counter >= 13) {
			_decode_pvt_header(&_buf[3]);
			_step++;
		}
		break;

	case STATE_DATA:
		if (_counter >= (sizeof(_buf) - 3)) {
			_decode_pvt_coo(&_buf[13]);
			_decode_pvt_vel(&_buf[39]);
			_step++;
		}
		break;

	case STATE_CRC:
		if (_counter >= sizeof(_buf)) {
			_decode_crc(&_buf[sizeof(_buf) - 3]);
			// TODO: CRC Check
			//if (crc24q())
			{
				parsed = _decode();
			}
			_step = STATE_PREAMBLE;
		}
		break;

	default:
		//PX4_INFO("Step Error!");
		_step = STATE_PREAMBLE;
		break;
	}

	return parsed;
}

void
AP_GPS_ATOM::_request()
{
	uint8_t str_clear[23] = "$PASHS,ATM,PVT,A,OFF\r\n";
	uint8_t str_request[36] = "$PASHS,ATM,PVT,A,ON,0.05,&COO,VEL\r\n";

	port->write(str_clear, sizeof(str_clear));
	port->write(str_request, sizeof(str_request));
}

void
AP_GPS_ATOM::inject_data(uint8_t *data, uint8_t len)
{

    if (port->txspace() > len) {
        port->write(data, len);
    } else {
        Debug("ATOM: Not enough TXSPACE");
    }
}

AP_GPS::GPS_Status
AP_GPS_ATOM::_get_gps_status()
{
	if(_pvt.coo.pos_mode != 0)
		return AP_GPS::NO_FIX;

	switch(_pvt.coo.pos_type)
	{
	case 1:
		return AP_GPS::GPS_OK_FIX_3D;

	case 2:
		return AP_GPS::GPS_OK_FIX_3D_DGPS;

	case 5:
		return AP_GPS::GPS_OK_FIX_3D_DGPS;

	case 4:
		return AP_GPS::GPS_OK_FIX_3D_RTK;
	}

	return AP_GPS::NO_FIX;
}

void
AP_GPS_ATOM::_convert_pos_ecef_to_llh()
{
	_pos_ecef.x = _pvt.coo.x / 10000.0;
	_pos_ecef.y = _pvt.coo.y / 10000.0;
	_pos_ecef.z = _pvt.coo.z / 10000.0;

	wgsecef2llh(_pos_ecef, _pos_llh);
}

/**
 * @remark refer to http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
 */
void
AP_GPS_ATOM::_calculate_ecef2ned(double lat, double lon)
{
	double lat_rad = radians(lat);
	double lon_rad = radians(lon);

	double clat = cos(lat_rad);
	double slat = sin(lat_rad);
	double clon = cos(lon_rad);
	double slon = sin(lon_rad);

	// ECEF -> N
	_ecef2ned[0][0] = -clon*slat;
	_ecef2ned[0][1] = -slon*slat;
	_ecef2ned[0][2] = clat;

	// ECEF -> E
	_ecef2ned[1][0] = -slon;
	_ecef2ned[1][1] = clon;
	_ecef2ned[1][2] = 0;

	// ECEF -> D
	_ecef2ned[2][0] = -clon*clat;
	_ecef2ned[2][1] = -slon*clat;
	_ecef2ned[2][2] = -slat;


}

void
AP_GPS_ATOM::_convert_vel_ecef_to_ned()
{
	double v_ecef_x = _pvt.vel.vx * 0.0001;
	double v_ecef_y = _pvt.vel.vy * 0.0001;
	double v_ecef_z = _pvt.vel.vz * 0.0001;

	_vel_ned.x = 		_ecef2ned[0][0] * v_ecef_x
						+	_ecef2ned[0][1] * v_ecef_y
						+ 	_ecef2ned[0][2] * v_ecef_z;

	_vel_ned.y = 		_ecef2ned[1][0] * v_ecef_x
						+	_ecef2ned[1][1] * v_ecef_y
						+ 	_ecef2ned[1][2] * v_ecef_z;

	_vel_ned.z = 		_ecef2ned[2][0] * v_ecef_x
						+	_ecef2ned[2][1] * v_ecef_y
						+ 	_ecef2ned[2][2] * v_ecef_z;
}

static const uint32_t crc24qtab[256] = { 0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6,
		0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17, 0xA18139, 0x27CDC2, 0x2B5434,
		0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E, 0xC54E89, 0x430272,
		0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E, 0x64CFB0,
		0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7,
		0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205,
		0x93AEFE, 0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA,
		0xB4633C, 0x322FC7, 0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981,
		0xDC357A, 0xD0AC8C, 0x56E077, 0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF,
		0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E, 0x19A3D2, 0x9FEF29, 0x9376DF,
		0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5, 0xB822EB, 0x3E6E10,
		0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC, 0xDCED5B,
		0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
		0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E,
		0xE21375, 0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821,
		0x0C41D7, 0x8A0D2C, 0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3,
		0xA15918, 0xADC0EE, 0x2B8C15, 0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544,
		0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5, 0x71BD8B, 0xF7F170, 0xFB6886,
		0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C, 0x3347A4, 0xB50B5F,
		0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3, 0x92C69D,
		0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
		0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1,
		0x69763A, 0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E,
		0x4EBBF8, 0xC8F703, 0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC,
		0x2A3C57, 0x26A5A1, 0xA0E95A, 0x9E1774, 0x185B8F, 0x14C279, 0x928E82,
		0x0DF195, 0x8BBD6E, 0x872498, 0x016863, 0xFAD8C4, 0x7C943F, 0x700DC9,
		0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3, 0x5B59FD, 0xDD1506,
		0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA, 0x2AE476,
		0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61,
		0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3,
		0x141A58, 0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5,
		0xF69913, 0x70D5E8, 0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27,
		0x5B81DC, 0x57182A, 0xD154D1, 0x26359F, 0xA07964, 0xACE092, 0x2AAC69,
		0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88, 0x87B4A6, 0x01F85D, 0x0D61AB,
		0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1, 0xE37B16, 0x6537ED,
		0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401, 0x42FA2F,
		0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538 };


