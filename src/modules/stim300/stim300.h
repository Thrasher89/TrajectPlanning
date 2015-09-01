/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Author: Stephan Zuercher stephan.zuercher@stud.hslu.ch
 */


#ifndef _STIM300_H_
#define _STIM300_H_

#define STREAM_MPU600_no

#ifdef STREAM_MPU600
#include <uORB/uORB.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <fcntl.h>
#endif

//GPS stuff
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_gps_heading.h>

#include "drivers/gps/novatel.h"
#include <uORB/topics/novatel_bestpos.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_bodyframe_speed.h>
#include <uORB/topics/vehicle_bodyframe_speed_setpoint.h>
#include <uORB/topics/wePilot.h>

#include <sched.h>
#include "serial.h"

namespace stim300
{

	/**
	 *Standard gravity constant
	 */
	#define STIM300_ONE_G	9.80665f

	/**
	 * STIM States Init => starting up; NORMAL => data streaming; Service => listen for configuration  input
	 */
	enum STIM300_MODE{NOT_CONNECTED=0, INIT, NORMAL, SERVICE};

	enum dataGramms {
		PART_NO		= 0xB3, 
		SERIAL_NO	= 0xB7, 
		CONFIG		= 0xBD, 
		NORMAL_MODE_93	= 0x93,
		wePilot = 0x00
	};

	enum stimState{
		uninitialized = 0,
		startingUp,
		idle,
		streaming,
	};



	class Stim300 : protected Serial
	{
		public:
			/**
			  *Singelton Class
			  */
			static Stim300* Instance();
			static void Start();

			void readThread();
			static int readThreadTrampolin(const char**);
			void getInfo();

			static uint32_t stim300_crc32(uint8_t *data, int len, int dummyBytes);
			static uint8_t wePilot_crc(uint8_t *data, int len);
			static int32_t get32bit(uint8_t msb, uint8_t midsb, uint8_t lsb);
			static uint8_t conv8bit2uint8(uint8_t msb);
			static int8_t conv8bit2int8(uint8_t msb);
			static uint16_t conv16bit2uint16(uint8_t msb, uint8_t lsb);
			static int16_t conv16bit2int16(uint8_t msb, uint8_t lsb);
			static float conv24bit2float(uint8_t msb, uint8_t midb, uint8_t lsb, unsigned int divideBy);
			static uint32_t conv32bit2uint32(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb);
			static int32_t conv32bit2int32(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb);
			static float conv32bit2float(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb);
			static char* getState();

		private:
			void startUp();
			stimState state = uninitialized;
			/**
			  *Constructor
			  */
			Stim300();
			/**
			  *Deconstructor
			  */
			~Stim300();
			Stim300(Stim300 const&);
			Stim300 operator=(Stim300 const&);
			uint8_t* getDatagramm(dataGramms);
			uint8_t* getDatagrammWePilot(dataGramms);
			uint8_t* getDatagrammWePilotFast(dataGramms);
			unsigned getDatagrammSize(dataGramms type);

			unsigned datagrammReadCnt=0;
			unsigned datagrammReadErrCnt=0;
			unsigned datagrammCRCErrCnt=0;

			uint8_t dataBuffer[60];
			uint8_t dataBufferWePilot[85];

			/* create topic metadata */
			ORB_DEFINE(vehicle_bodyframe_speed, struct vehicle_bodyframe_speed_s);
			ORB_DEFINE(wePilot, struct wePilot_s);

			int		_global_position_sub;
			int		_local_position_sub;
			int		_attitude_sub;
			int		_bodyframe_speed_sub;
			int		_bodyframe_speed_setpoint_sub;
			int		_wePilot_sub;

			orb_advert_t	_global_position_pub;			/**< Global Position */
			orb_advert_t	_local_position_pub;			/**< Local Position */
			orb_advert_t	_attitude_pub;					/**< Attitude */
			orb_advert_t	_bodyframe_speed_pub;			/**< Vehicle Bodyframe Speed */
			orb_advert_t	_bodyframe_speed_pub_setpoint;	/**< Vehicle Bodyframe Speed Setpoint*/
			orb_advert_t	_wePilot_pub;					/**< wePilot */

			struct vehicle_global_position_s _global_position;
			struct vehicle_local_position_s _local_position;
			struct vehicle_attitude_s _attitude;
			struct vehicle_bodyframe_speed_s _bodyframe_speed;
			struct vehicle_bodyframe_speed_s _bodyframe_speed_setpoint;
			struct wePilot_s _wePilot;


		protected:
//			struct p_int24_t {
//				signed int:24;
//			}__attribute__ ((packed)); // packed: dont allow to make a 32bit pack (padding) on 32bit system. Its 24bit

//			struct p_int16_t {
//				signed int:16;
//			}__attribute__ ((packed));
	
			void putCmd(char *);

			/**
			  * Serialport speed
			  */
			int baudrate; 

			/**
			  * Sensors sampling frequenz
			  */
			int sampling_rate;

			/**
			  * Sensors state
			  */
			STIM300_MODE mode;

			Serial out;

#ifdef STREAM_MPU600 
			void board_accel_init(void);
#endif
	




	}; 





struct s_datagram_partNo{
	uint8_t id;
	uint8_t partNo1[3];
	uint8_t delimiter1;
	uint8_t partNo2[3];
	uint8_t delimiter2;
	uint8_t partNo3[2];
	uint8_t noUsed[4];
	uint8_t revision;
	uint8_t crc[4];
	uint8_t cr;
	uint8_t lf;
};

struct s_datagram_serialNo{
	uint8_t id;
	uint8_t data[15];
	uint8_t crc[4];
	uint8_t cr; 
	uint8_t lf; 
};

struct s_datagram_config{
	uint8_t id;
	uint8_t data[21];
	uint8_t crc[4];
	uint8_t cr; 
	uint8_t lf; 
};

struct s_datagram_normalMode_93{
	uint8_t id; 
	uint8_t gyroX[3];
	uint8_t gyroY[3];
	uint8_t gyroZ[3];
	uint8_t gyroStat;

	uint8_t accX[3];
	uint8_t accY[3];
	uint8_t accZ[3];
	uint8_t accStat;

	uint8_t incX[3];
	uint8_t incY[3];
	uint8_t incZ[3];
	uint8_t incStat;

	uint8_t cnt;
	uint8_t lat[2];
	uint8_t crc[4];
	uint8_t cr; 
	uint8_t lf; 
};

struct s_datagram_wePilot{
// ------------------------------------------------------------------------------------------------------
//	Name					Offset	Type	Scaling		Unit 	Range 		Description
// ------------------------------------------------------------------------------------------------------

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
};




}//end namespace

#endif
