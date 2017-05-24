#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_GPS.h"
#include <string.h>

class AP_GPS_ATOM : public AP_GPS_Backend
{
public:
	AP_GPS_ATOM(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    // Methods
    bool read();

    AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D_RTK; }

    static bool _detect(struct ATOM_detect_state &state, uint8_t data);

private:
    typedef enum {
    	STATE_PREAMBLE,
    	STATE_LENGTH,
    	STATE_HEADER,
    	STATE_DATA,
    	STATE_CRC,
    }atom_state_machine;

    typedef struct {
    	uint8_t	preamble : 8;
    	uint8_t	reserved : 6;
    	uint16_t	msg_length : 10;
    }PACKED atom_start;

    typedef struct {
    	uint16_t	msg_number : 12;

    	uint8_t		msg_sub_number : 4;
    	uint8_t		version : 3;
    	uint8_t		multiple_msg_bit : 1;

    	uint8_t		antenna_id : 4;
    	uint8_t		engine_id : 6;
    	uint8_t		response_id : 4;
    	uint8_t		reserved : 5;
    	uint8_t		nsats_used : 6;
    	uint8_t		nsats_seen : 6;
    	uint8_t		nsats_tracked : 6;
    	uint8_t		primary_gnss : 2;
    	uint32_t	time_tag : 21;
    }PACKED atom_pvt_header;

    typedef struct {
    	uint8_t		block_size : 8;
    	uint8_t		block_id : 4;
    	uint8_t		pos_type : 4;
    	uint8_t		gnss_mask : 8;
    	uint8_t		pos_mode : 3;
    	uint8_t		pos_smoothing : 3;
    	uint8_t		reserved : 4;
    	uint16_t	pdop : 10;
    	uint16_t	hdop : 10;
    	int64_t		x : 38;
    	int64_t		y : 38;
    	int64_t		z : 38;
    	uint16_t	diff_age : 10;
    	uint16_t	base_id : 12;
    	uint8_t		pos_type_clarifier : 4;
    	uint16_t	diff_link_age : 10;
    	uint8_t		reserved2 : 4;
    }PACKED atom_coo;

    typedef struct {
    	uint8_t		block_size : 8;
    	uint8_t		block_id : 4;
    	int32_t		vx : 25;
    	int32_t		vy : 25;
    	int32_t		vz : 25;
    	uint8_t		vel_type : 1;
    	uint8_t		doppler_smoothing : 4;
    	uint8_t		vel_frame : 1;
    	uint8_t		reserved : 3;
    }PACKED atom_vel;

    typedef struct {
    	uint32_t	crc : 24;
    }PACKED atom_crc;

    typedef struct {

		atom_start start;
		atom_pvt_header header;
		atom_coo coo;
		atom_vel vel;
		atom_crc crc;
    }PACKED atom_pvt;

    static const uint32_t crc24qtab[256];
    int _step = STATE_PREAMBLE;
    uint8_t _counter = 0;
    atom_pvt _pvt;
    uint8_t _buf[sizeof(atom_pvt)];

    // ECEF to NED Transformation Matrix
	double _ecef2ned[3][3];

	Vector3d _pos_ecef;
	Vector3d _pos_llh;
	Vector3d _vel_ned;

	// do we have new position information?
	bool _new_position:1;
	// do we have new speed information?
	bool _new_speed:1;

    uint32_t crc24q(const uint8_t *buf, uint32_t len, uint32_t crc);
    uint32_t crc24q_bits(uint32_t crc, const uint8_t *buf, uint32_t n_bits, bool invert);
    void _decode_start(uint8_t buf[]);
    void _decode_pvt_header(uint8_t buf[]);
    void _decode_pvt_coo(uint8_t buf[]);
    void _decode_pvt_vel(uint8_t buf[]);
    void _decode_crc(uint8_t buf[]);
    bool _decode();
    bool _parse(uint8_t data);
    void _request();
    void _calculate_ecef2ned(double lat, double lon);
    void _convert_pos_ecef_to_llh();
    void _convert_vel_ecef_to_ned();
    AP_GPS::GPS_Status _get_gps_status();
    void inject_data(uint8_t *data, uint8_t len);





};
