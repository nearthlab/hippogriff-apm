#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_GPS.h"
#include <string.h>

class AP_GPS_NOVATEL : public AP_GPS_Backend
{
public:
    AP_GPS_NOVATEL(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    // Methods
    bool read();

    AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D_RTK; }

    static bool _detect(struct NOVATEL_detect_state &state, uint8_t data);

private:
    struct PACKED novatel_header {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t preamble3;
        uint8_t header_length;

        uint16_t message_id;
        int8_t message_type;
        uint8_t port_address;

        uint16_t message_length;
        uint16_t sequence;

        uint8_t idle_time;
        uint8_t time_status;
        uint16_t week;

        uint32_t seconds;

        uint32_t receiver_status;

        uint16_t reserved;
        uint16_t receiver_version;
    };

    struct PACKED novatel_bestposb {
    	uint32_t	sol_stat;
    	uint32_t	pos_type;
    	double   	lat;
    	double		lon;
    	double		hgt;
    	float 		undulation;
    	uint32_t	datum_id_n;
    	float		lat_sigma;
    	float		lon_sigma;
    	float		hgt_sigma;
    	int8_t		stn_id[4];
    	float		diff_age;
    	float		sol_age;
    	uint8_t	    n_sv;
    	uint8_t	    n_sol_sv;
    	uint8_t	    n_sol_l1_sv;
    	uint8_t	    n_sol_multi_sv;
    	uint8_t	    reserved;
    	uint8_t	    ext_sol_stat;
    	uint8_t	    galileo_beidou_sig_mask;
    	uint8_t	    gps_glonass_sig_mask;
    };

    struct PACKED novatel_bestvelb {
		uint32_t	sol_status;
		uint32_t 	vel_type;
		float	 	latency;
		float		age;
		double		hor_spd;
		double		trk_gnd;
		double		vert_spd;
		float		reserved;
	};

    // Header
	typedef union PACKED {
		novatel_header data;
		uint8_t bytes[];
	} novatel_header_union;

    // Message
	typedef union PACKED {
		novatel_bestposb bestposb;
		novatel_bestvelb bestvelb;
		uint8_t bytes[];
	} novatel_message_union;

	// Packet
	typedef struct PACKED {
		novatel_header_union hdr;
		novatel_message_union msg;
	} novatel_packet;

    enum novatel_preamble {
        PREAMBLE1 = 0xAA,
        PREAMBLE2 = 0x44,
        PREAMBLE3 = 0x12,
    };

    enum novatel_message_id{
        MSG_BESTPOSB = 42,
        MSG_BESTVELB = 99,
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

    enum novatel_state_machine{
    	STATE_PREAMBLE1,
    	STATE_PREAMBLE2,
    	STATE_PREAMBLE3,
    	STATE_GET_HEADER,
    	STATE_GET_DATA,
    	STATE_CHECKSUM1,
    	STATE_CHECKSUM2,
    	STATE_CHECKSUM3,
    	STATE_CHECKSUM4,
    	STATE_OK
    };

    novatel_packet _packet;

    // Packet checksum accumulators
    uint8_t _ck_a;
    uint8_t _ck_b;
    uint8_t _ck[4];

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

    // Buffer parse & GPS state update
    bool _parse_gps();

    void inject_data(uint8_t *data, uint8_t len);

    // used to update fix between status and position packets
    AP_GPS::GPS_Status next_fix;

    bool _check_checksum();
    uint32_t _calculate_crc32(int32_t i);
    uint32_t _calculate_block_crc32(uint32_t ulCount, uint8_t *ucBuffer);
    AP_GPS::GPS_Status _convert_gps_status(uint32_t pos_type);
    void _fill_ned_vel(double hor_spd, double trk_gnd, double vert_spd);
};
