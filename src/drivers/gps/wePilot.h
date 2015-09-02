/*
 * drv_wePilot.h
 *
 *  Created on: 02.09.2015
 *      Author: tetresch
 */

#ifndef DRV_WEPILOT_H_
#define DRV_WEPILOT_H_

#include "gps_helper.h"

#define WEPILOT_SYNC 0xA5
#define WEPILOT_SYNC2 0x44
#define WEPILOT_SYNC3 0x12
/* MessageIDs (the ones that are used) */
#define WEPILOT_ID 0x00
#define WEPILOT_MsgLength 0x52
#define SEND55 0x55
#define SENDA5 0x00
#define CRCINDEX 86
#define WEPILOT_BESTPOS 42
#define WEPILOT_BESTVEL 99
#define WEPILOT_PSRPOS	47
#define WEPILOT_PSRDOP	174
#define WEPILOT_HEADING 971

#define CRC32_POLYNOMIAL 0xEDB88320


#define WEPILOT_UNLOGALL		"UNLOGALL\r\n"
#define WEPILOT_BESTPOS_5HZ		"log bestposb ontime 0.2\r\n"
#define WEPILOT_BESTVEL_5HZ		"log bestvelb ontime 0.2\r\n"
#define WEPILOT_HEADING_5HZ		"log headingb ONCHANGED\r\n"
//#define SBAS_ON	        		"sbascontrol enable auto\r\n"
//#define MAGVAR_ZERO				"magvar correction 0 0\r\n"

#define WEPILOT_BAUDRATE 57600
#define WEPILOT_WAIT_BEFORE_READ_USEC 1


#define WEPILOT_FIXTYPE_NONE 0 				// No solution
#define WEPILOT_FIXTYPE_FIXEDPOS 1 			// Position has been fixed by the FIX POSITION command
#define WEPILOT_FIXTYPE_FIXEDHEIGHT 2 		// Position has been fixed by the FIX HEIGHT/AUTO command
#define WEPILOT_FIXTYPE_DOPPLER_VELOCITY 8  // Velocity computed using instantaneous Doppler
#define WEPILOT_FIXTYPE_SINGLE 16 			// Single point position
#define WEPILOT_FIXTYPE_PSRDIFF 17 			// Pseudorange differential solution
#define WEPILOT_FIXTYPE_WAAS 18 			// Solution calculated using corrections from an SBAS
#define WEPILOT_FIXTYPE_PROPAGATED 19 		// Propagated by a Kalman filter without new observations
#define WEPILOT_FIXTYPE_OMNISTAR 20 		// a OmniSTAR VBS position (L1 sub-metre)
#define WEPILOT_FIXTYPE_L1_FLOAT 32 		// Floating L1 ambiguity solution
#define WEPILOT_FIXTYPE_IONOFREE_FLOAT 33 	// Floating ionospheric-free ambiguity solution
#define WEPILOT_FIXTYPE_NARROW_FLOAT 34 	// Floating narrow-lane ambiguity solution
#define WEPILOT_FIXTYPE_L1_INT 48 			// Integer L1 ambiguity solution
#define WEPILOT_FIXTYPE_WIDE_INT 49 		// Integer wide-lane ambiguity solution
#define WEPILOT_FIXTYPE_NARROW_INT 50 		// Integer narrow-lane ambiguity solution
#define WEPILOT_FIXTYPE_RTK_DIRECT_INS 51 	// RTK status where the RTK filter is directly initialized from the INS filter
#define WEPILOT_FIXTYPE_INS 52 				// INS calculated position corrected for the antenna
#define WEPILOT_FIXTYPE_INS_PSRSP 53 		// INS pseudorange single point solution - no DGPS corrections
#define WEPILOT_FIXTYPE_INS_PSRDIFF 54 		// INS pseudorange differential solution
#define WEPILOT_FIXTYPE_INS_RTKFLOAT 55 	// INS RTK floating point ambiguities solution
#define WEPILOT_FIXTYPE_INS_RTKFIXED 56 	// INS RTK fixed ambiguities solution
#define WEPILOT_FIXTYPE_OMNISTAR_HP 64 		// OmniSTAR HP position
#define WEPILOT_FIXTYPE_OMNISTAR_XP 65 		// OmniSTAR XP position
#define WEPILOT_FIXTYPE_CDGPS 66 			// Position solution using CDGPS correction



typedef enum {
	WEPILOT_DECODE_UNINIT = 0,
	WEPILOT_DECODE_GOT_SYNC,
	WEPILOT_DECODE_GOT_ID,
//	WEPILOT_DECODE_GOT_SYNC2,
//	WEPILOT_DECODE_GOT_SYNC3,
//	WEPILOT_DECODE_GOT_HEADER_LGTH,
	WEPILOT_DECODE_GOT_MESSAGE_LGTH,
	WEPILOT_DECODE_GOT_MESSAGE,
	WEPILOT_DECODE_GOT_CRC,
//	WEPILOT_DECODE_GOT_CRC2,
//	WEPILOT_DECODE_GOT_CRC3
} WEPILOT_decode_state_t;

typedef enum {
	WEPILOT_TIME_STATUS_UNKNOWN = 20,
	WEPILOT_TIME_STATUS_APPROXIMATE = 60,
	WEPILOT_TIME_STATUS_COARSEADJUSTING = 80,
	WEPILOT_TIME_STATUS_COARSE = 100,
	WEPILOT_TIME_STATUS_COARSESTEERING = 120,
	WEPILOT_TIME_STATUS_FREEWHEELING = 130,
	WEPILOT_TIME_STATUS_FINEADJUSTING = 140,
	WEPILOT_TIME_STATUS_FINE = 160,
	WEPILOT_TIME_STATUS_FINESTEERING = 180,
	WEPILOT_TIME_STATUS_SATTIME = 200
} WEPILOT_gps_time_status;


/** the structures of the binary packets */
#pragma pack(push, 1)

typedef struct {
	// ------------------------------------------------------------------------------------------------------
	//	Name					Offset	Type	Scaling		Unit 	Range 		Description
	// ------------------------------------------------------------------------------------------------------
		uint8_t sync;		//													Synchronizer (0xA5)

		uint8_t id;			//													Identifier (0x00)

		uint8_t length;		//													Message Payload length (52)

		uint8_t latPos[4];	//	0 		I4 		1E-8 		rad 	- 			Latitude (COG)
		uint8_t lonPos[4];	//	4 		I4 		1E-8 		rad 	- 			Longitude (COG)
		uint8_t altPos[4];	//	8 		I4 		1E-3 		m 		- 			Altitude mean sea level (COG)

		uint8_t relPosN[4];	//	12 		I4 		1E-2 		m 		- 			Relative position North (COG)
		uint8_t relPosE[4];	//	16 		I4 		1E-2 		m 		- 			Relative positon East (COG)
		uint8_t relPosD[4];	//	20 		I4 		1E-2 		m 		- 			Relative position Down (COG)

		uint8_t gndVelN[4];	//	24 		R4 		-			m/s 	- 			Ground velocity North (COG)
		uint8_t gndVelE[4];	//	28 		R4 		-			m/s 	- 			Ground velocity East (COG)
		uint8_t gndVelD[4];	//	32 		R4 		-			m/s 	- 			Ground velocity Down (COG)

		uint8_t fX[4];		//	36 		R4 		- 			m/s2 	- 			Specific force x-body axis (longitudinal)
		uint8_t fY[4];		//	40 		R4 		- 			m/s2 	- 			Specific force y-body axis (lateral)
		uint8_t fZ[4];		//	44 		R4 		- 			m/s2 	- 			Specific force z-body axis (vertical)

		uint8_t roll[2];	//	48 		I2 		1E-4 		rad 	+-3.1415 rad Roll
		uint8_t pitch[2];	//	50 		I2 		1E-4 		rad 	+-3.1415 rad Pitch
		uint8_t yaw[2];		//	52 		I2 		1E-4 		rad 	+-3.1415 rad Yaw

		uint8_t p[2];		//	54 		I2 		1E-3 		rad/s 	+-31.415 rad/s Angular rate x-body axis (longitudinal)
		uint8_t q[2];		//	56 		I2 		1E-3 		rad/s 	+-31.415 rad/s Angular rate x-body axis (lateral))
		uint8_t r[2];		//	58 		I2 		1E-3 		rad/s 	+-31.415 rad/s Angular rate x-body axis (vertical))

		uint8_t hx[2];		//	60 		I2 		- 			- 		- 			Earth magnetic field x-body axis
		uint8_t hy[2];		//	62 		I2 		- 			- 		- 			Earth magnetic field y-body axis
		uint8_t hz[2];		//	64 		I2 		- 			- 		- 			Earth magnetic field z-body axis

		uint8_t rotor[2];	//	66 		U2 		- 			rpm 	0...65535 	rpm Current rotor frequency

		uint8_t time[4];	//	68 		U4 		- 			- 		-			GPS Time:
		//																		Bit 15-0: Seconds, scaled with 1000 (0-
		//																		59999)
		//																		Bit 23-16: Minutes (0-59)
		//																		Bit 31-24: Hours (0-23)

		uint8_t status[2];	//	72 		U2 		- 			- 		- 			Status:
		//																		Bit 3-0: FCS State
		//																		0=NOT READY, 1=READY, 2=TESTSERVOS, 3=TAKEOFF, 4=LANDING, 5=SHUTDOWNENGINE, 6=HOVER, 7=CRUISE, 8=GOTO WP, 9=STOP AT WP, 10=GOTO HOME, 11=STOP AT HOME, 12=LINKLOST
		//																		Bit 4: PWM INP7, 0=off, 1=on
		//																		Bit 11-8: Number of valid GPS sat

		uint8_t refU[2];	//	74 		I2 		1E-3 		m/s 	-4...10 m/s Total reference longitudinal speed
		uint8_t refV[2];	//	76 		I2 		1E-3 		m/s 	-4...4 m/s 	Total reference lateral speed
		uint8_t refW[2];	//	78 		I2 		1E-3 		m/s 	-2...2 m/s 	Total reference descent rate
		uint8_t refR[2];	//	80 		I2 		1E-3 		°/s 	-30...30 °/s Total reference heading rate

		uint8_t crc;		//													Checksum
	}gps_WEPILOT_datagram;

typedef struct {
	uint32_t solstat; /**< Solution status */
	uint32_t postype; /**< Position type */
	double_t lat; /**< Latitude, deg */
	double_t lon; /**< Longitude, deg */
	double_t hgt; /**< Height above mean sea level, m */
	float undulation; /**< Undulation - the relationship between the geoid and the ellipsoid (m) of the chosen datum */
	uint32_t datum_id_nr; /**< Datum ID number */
	float sigma_lat; /**< Latitude standard deviation, deg */
	float sigma_lon; /**< Longitude standard deviation, deg */
	float sigma_hgt; /**< Height standard deviation, m */
	uint32_t stn_id; /**< Base station ID */
	float dgps_age; /**< Differential age, s */
	float sol_age; /**< Solution age, s */
	uint8_t sat_tracked; /**< Number of satellite vehicles tracked */
	uint8_t sat_used; /**< Number of satellite vehicles used in solution */
	uint8_t ggL1; /**< Number of GPS plus GLONASS L1 used in solution */
	uint8_t ggL1L2; /**< Number of GPS plus GLONASS L1 and L2 used in solution */
	uint8_t reserved_1; /**< Reserved */
	uint8_t ext_sol_stat; /**< Extended solution status */
	uint8_t reserved_2; /**< Reserved */
	uint8_t sig_mask; /**< Signals used mak - if 0, signals used in solution are unknown */
} gps_WEPILOT_bestpos_packet_t;

typedef struct {
	uint32_t solstat; /**< Solution status */
	uint32_t veltype; /**< Velocity type */
	float latency; /**< A measure of the latency in the velocity time tag in seconds. It should be subtracted from the time to give improved results */
	float dgps_age; /**< Differential age, s */
	double_t hor_spd; /**< Horizontal speed over ground, in meters per second */
	double_t trk_gnd; /**< Actual direction of motion over ground (track over ground) with respect to True North, in degrees */
	double_t vert_spd; /**< Vertical speed, in meters per second, where positive values indicate increasing altitude (up) and negative values indicate decreasing altitude (down) */
	float reserved; /**< Reserved */
} gps_WEPILOT_bestvel_packet_t;


typedef struct {
	uint64_t mc_time;
	uint16_t week;
	uint32_t ms;
	uint8_t timeState;
} gps_WEPILOT_time_t;

typedef struct {
	uint32_t solstat;
	uint32_t postype;
	float length;
	float heading;
	float pitch;
	float reserved;
	float hdg_std_dev;
	float ptch_std_dev;
	uint8_t stationName[4];
	uint8_t num_sat_track;
	uint8_t num_sat_solution;
	uint8_t num_sat_above_elevation;
	uint8_t num_sat_above_L2;
	uint8_t sol_src;
	uint8_t ext_sol_stat;
	uint8_t Galileo_BeiDou_mask;
	uint8_t GPS_mask;
} gps_WEPILOT_heading_packet_t;


typedef struct {
	float gdop; /**< Geometric dilution of precision */
	float pdop; /**< Position dilution of precision */
	float hdop; /**< Horizontal dilution of precision */
	float htdop; /**< Horizontal position and time dilution of precision*/
	float tdop; /**< Time dilution of precision */
	float cutoff; /**< Elevation cut-off angle*/
	long	Number_PRN; /**< Number of satellites PRNs to follow*/
	unsigned long PRN1;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN2;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN3;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN4;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN5;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN6;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN7;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN8;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN9;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN10;	/**< only 10 PRN implemented at the moment*/
} gps_WEPILOT_psrdop_packet_t;


typedef struct {
	// todo implement it



	uint32_t solstat; /**< Solution status */
	uint32_t veltype; /**< Velocity type */
	float latency; /**< A measure of the latency in the velocity time tag in seconds. It should be subtracted from the time to give improved results */
	float dgps_age; /**< Differential age, s */
	double_t hor_spd; /**< Horizontal speed over ground, in meters per second */
	double_t trk_gnd; /**< Actual direction of motion over ground (track over ground) with respect to True North, in degrees */
	double_t vert_spd; /**< Vertical speed, in meters per second, where positive values indicate increasing altitude (up) and negative values indicate decreasing altitude (down) */
	float reserved; /**< Reserved */
} gps_WEPILOT_psrpos_packet_t;



//#include <uORB/topics/novatel_bestpos.h>
//#include <uORB/topics/novatel_bestvel.h>
//#include <uORB/topics/novatel_heading.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_bodyframe_speed.h>
#include <uORB/topics/vehicle_bodyframe_speed_setpoint.h>
#include <uORB/topics/wePilot_info.h>

#pragma pack(pop)

#define WEPILOT_RECV_BUFFER_SIZE 512

class WEPILOT : public GPS_Helper
{
public:
//	WEPILOT(const int &fd, struct vehicle_gps_position_s *gps_position, struct satellite_info_s *satellite_info,struct vehicle_gps_heading_s *gps_heading);
	WEPILOT(const int &fd);
	~WEPILOT();
	int				receive(unsigned timeout);
	int				configure(unsigned &baudrate);

private:
	/**
	 * Parse the binary WEPILOT packet
	 */
	int				parse_char(uint8_t b);

	/**
	 * Handle the package once it has arrived
	 */
	int				handle_message(void);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void				decode_init(void);

	/**
	 * Calculate the CRC-32 of a block of data
	 */

	uint8_t				calculate_block_wePilot(unsigned long message_lgth, unsigned char *data);
	unsigned long		calculate_block_crc32(unsigned long message_lgth, unsigned char *data);
	unsigned long 		CRC32Value(int i);
	static uint8_t 		conv8bit2uint8(uint8_t msb);
	static int8_t 		conv8bit2int8(uint8_t msb);
	static uint16_t 	conv16bit2uint16(uint8_t msb, uint8_t lsb);
	static int16_t 		conv16bit2int16(uint8_t msb, uint8_t lsb);
	static uint32_t 	conv32bit2uint32(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb);
	static int32_t 		conv32bit2int32(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb);
	static float 		conv32bit2float(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb);

	int					_fd;

	struct vehicle_gps_position_s *_gps_position;
	struct vehicle_gps_heading_s *_gps_heading;
	struct satellite_info_s *_satellite_info;

//	gps_novatel_time_t		_gps_time;
//	struct novatel_bestvel_s	_best_vel;
//	struct novatel_bestpos_s	_best_pos;
//	struct novatel_heading_s	_heading;

	struct vehicle_global_position_s _global_position;
	struct vehicle_local_position_s _local_position;
	struct vehicle_attitude_s _attitude;
	struct vehicle_bodyframe_speed_s _bodyframe_speed;
	struct vehicle_bodyframe_speed_setpoint_s _bodyframe_speed_setpoint;
	struct wePilot_info_s _wePilot_info;

//	orb_advert_t	_report_best_pos=0;
//	orb_advert_t	_report_best_vel=0;
//	orb_advert_t	_report_heading=0;

	orb_advert_t	_global_position_pub;			/**< Global Position */
	orb_advert_t	_local_position_pub;			/**< Local Position */
	orb_advert_t	_attitude_pub;					/**< Attitude */
	orb_advert_t	_bodyframe_speed_pub;			/**< Vehicle Bodyframe Speed */
	orb_advert_t	_bodyframe_speed_setpoint_pub;	/**< Vehicle Bodyframe Speed Setpoint*/
	orb_advert_t	_wePilot_info_pub;					/**< wePilot */

	WEPILOT_decode_state_t	_decode_state;
	uint8_t				_send55Flag;
	uint8_t				_WEPILOT_revision;
	uint8_t				_rx_header_lgth;
	uint16_t			_rx_message_lgth;
	uint16_t			_rx_message_id;
	uint8_t				_rx_buffer[WEPILOT_RECV_BUFFER_SIZE];
	uint8_t				_crc_buffer[4];
	uint32_t			_rx_count;
	uint32_t 			_rx_crc;
	uint8_t				_calculated_crc;
	WEPILOT_gps_time_status		_rx_message_time_status;
	uint16_t 			_rx_message_week;
	uint32_t			_rx_message_ms;
	bool			_got_posllh;
	bool			_got_velned;
};


#endif /* DRV_WEPILOT_H_ */
