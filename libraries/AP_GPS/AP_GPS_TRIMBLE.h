#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_GPS.h"
#include <string.h>

class AP_GPS_TRIMBLE : public AP_GPS_Backend
{
public:
    AP_GPS_TRIMBLE(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    // Methods
    bool read();

    AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D_RTK; }

    static bool _detect(struct NOVATEL_detect_state &state, uint8_t data);

private:
    typedef struct {
    	uint8_t new_position : 1;
    	uint8_t clock_fix : 1;
    	uint8_t horizontal_fix : 1;
    	uint8_t height_fix : 1;
    	uint8_t reserved1 : 1;
    	uint8_t least_square_pos : 1;
    	uint8_t reserved2 : 1;
    	uint8_t filtered_l1_pseudorange : 1;
    	uint8_t diff_pos : 1;
    	uint8_t diff_pos_method1 : 1;
    	uint8_t diff_pos_method2 : 1;
    	uint8_t omnistar_solution : 1;
    	uint8_t pos_det : 1;
    	uint8_t network_rtk : 1;
    	uint8_t location_rtk : 1;
    	uint8_t beacon_dgps : 1;
    }PACKED trimble_position_flag;

    typedef struct {
    	uint8_t validity : 1;
    	uint8_t computation : 1;
    	uint8_t reserved : 6;
    }PACKED trimble_velocity_flag;

    typedef struct {
    	int8_t		stx;
    	int8_t		status;
    	int8_t		packet_type;
    	int8_t		length;
    }PACKED trimble_header;

    typedef struct {
    	int8_t		output_record_type;
    	int8_t		record_length;
    	int32_t		gps_time;
    	int16_t		gps_week_number;
    	int8_t		n_sv;
    	int16_t		position_flag;
    	int8_t		initialized_number;
    } PACKED trimble_time;

    typedef struct {
    	int8_t		output_record_type;
    	int8_t		record_length;
    	double   	latitude;
    	double		longitude;
    	double		height;
    }PACKED trimble_llh;

    typedef struct {
    	int8_t		output_record_type;
    	int8_t		record_length;
    	trimble_velocity_flag		velocity_flags;
    	float		speed;
    	float		heading;
    	float		vertical_velocity;
    }PACKED trimble_velocity;

    // Header
    typedef union  {
    	trimble_header data;
    	uint8_t bytes[sizeof(trimble_header)];
    }PACKED trimble_header_union;

    // Message
    typedef struct  {
    	int8_t		transmission_number;
    	int8_t		page_index;
    	int8_t		max_page_index;
    	trimble_time time;
    	trimble_llh llh;
    	trimble_velocity velocity;
    }PACKED trimble_message;

    // Packet
    typedef struct  {
    	trimble_header_union hdr;
    	trimble_message msg;
    	uint8_t msg_buf[sizeof(trimble_message)];
    }PACKED trimble_packet;

    enum trimble_preamble {
    	GSOF_STX = 0x02,
    	GSOF_ETX = 0x03,
    };

    enum trimble_message_id {
    	GSOF_TIME = 0x01,
    	GSOF_LLH = 0x02,
    	GSOF_VELOCITY = 0x08,
    };

    enum novatel_fixed_type {
    	FIX_NONE = 0,
    	FIX_FIXEDPOS = 1,
    	FIX_FIXEDHEIGHT = 2,
    	FIX_SINGLE = 16,
    	FIX_PSRDIFF = 17,
    	FIX_PROPAGATED = 19,
    	FIX_L1_FLOAT = 32,
    	FIX_IONOFREE_FLOAT = 33,
    	FIX_NARROW_FLOAT = 34,
    	FIX_L1_INT = 48,
    	FIX_NARROW_INT = 50,
    };

    enum novatel_solution_status {
    	SOL_SOL_COMPUTED = 0,
    	SOL_INSUFFICIENT_OBS = 1,
    	SOL_NO_CONVERGENCE = 2,
    	SOL_SINGULARITY = 3,
    	SOL_COV_TRACE = 4,
    	SOL_COLD_START = 6,
    	SOL_V_H_LIMIT = 7,
    	SOL_INVALID_FIX = 19
    };

    enum novatel_state_machine {
    	STATE_PREAMBLE,
    	STATE_GET_HEADER,
    	STATE_GET_DATA,
    	STATE_CHECKSUM,
    	STATE_POSTAMBLE,
    	STATE_OK
    };

    trimble_packet _packet;

    // Packet checksum accumulators
    uint8_t _ck;

    // State machine state
    uint8_t _step;
    uint16_t _msg_id;
    uint16_t _payload_length;
    uint16_t _payload_counter;
    uint8_t _header_counter;

    // 8 bit count of fix messages processed, used for periodic processing
    uint8_t _fix_count;

    uint32_t _last_pos_time;
    uint32_t _last_vel_time;

    // do we have new position information?
    bool _new_position:1;
    // do we have new speed information?
    bool _new_speed:1;

    // ECEF to NED Transformation Matrix
    double _ecef2ned[3][3];

    void inject_data(uint8_t *data, uint8_t len);

    // used to update fix between status and position packets
	AP_GPS::GPS_Status next_fix;

    int32_t cvt_int(int32_t Src);
    float cvt_float(float Src);
    double cvt_double(double Src);

    bool read(uint8_t buffer[]);
    bool _check_checksum(void);
    AP_GPS::GPS_Status _convert_gps_status(int16_t position_flag);
    bool _is_vel_ok(trimble_velocity* p_vel);
    void _fill_ned_vel(double hor_spd, double heading, double vert_spd);
    bool _read_trimble_record(void);
    bool _parse_trimble(void);
};
