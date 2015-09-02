/*
 * wePilot.cpp
 *
 *  Created on: 02.09.2015
 *      Author: tetresch
 */

#include <assert.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>


#include <string.h>
#include <time.h>
#include <unistd.h>

#include <systemlib/err.h>
#include <uORB/uORB.h>
//#include <uORB/topics/vehicle_gps_position.h>
//#include <uORB/topics/vehicle_gps_heading.h>
//#include <uORB/topics/satellite_info.h>


//#include <uORB/topics/satellite_info.h>

#include <drivers/drv_hrt.h>


#include "wePilot.h"


//WEPILOT::WEPILOT(const int &fd, struct vehicle_gps_position_s *gps_position, struct satellite_info_s *satellite_info,struct vehicle_gps_heading_s *gps_heading ) :
WEPILOT::WEPILOT(const int &fd) :
_fd(fd)
//_gps_position(gps_position),
//_gps_heading(gps_heading),
//_satellite_info(satellite_info),
//_WEPILOT_revision(0)
{
	decode_init();
}

WEPILOT::~WEPILOT()
{
}

int
WEPILOT::configure(unsigned &baudrate)
{

	// set baudrate first
	if (GPS_Helper::set_baudrate(_fd, WEPILOT_BAUDRATE) != 0)
		return -1;

	baudrate = WEPILOT_BAUDRATE;

	/*
	printf("write config\n");
	// Write config messages, don't wait for an answer
	if (strlen(WEPILOT_UNLOGALL) != write(_fd, WEPILOT_UNLOGALL, strlen(WEPILOT_UNLOGALL))) {
		warnx("WEPILOT: config write failed");
		return -1;
	}
	usleep(3000);
	if (strlen(WEPILOT_HEADING_5HZ) != write(_fd, WEPILOT_HEADING_5HZ, strlen(WEPILOT_HEADING_5HZ))) {
		warnx("WEPILOT: config write failed");
		return -1;
	}
	usleep(1000);
	if (strlen(WEPILOT_BESTPOS_5HZ) != write(_fd, WEPILOT_BESTPOS_5HZ, strlen(WEPILOT_BESTPOS_5HZ))) {
		warnx("WEPILOT: config write failed");
		return -1;
	}
	usleep(1000);

	if (strlen(WEPILOT_BESTVEL_5HZ) != write(_fd, WEPILOT_BESTVEL_5HZ, strlen(WEPILOT_BESTVEL_5HZ))) {
		warnx("WEPILOT: config write failed");
		return -1;
	}
	usleep(1000);

	if (strlen(SBAS_ON) != write(_fd, SBAS_ON, strlen(SBAS_ON))) {
		warnx("WEPILOT: config write failed");
		return -1;
	}
	usleep(10000);

	if (strlen(MAGVAR_ZERO) != write(_fd, MAGVAR_ZERO, strlen(MAGVAR_ZERO))) {
		warnx("WEPILOT: config write failed");
		return -1;
	}
	usleep(10000);
*/


	return 0;
}

int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
WEPILOT::receive(unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	static uint8_t buf[128];

	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();

	static int j = 0;
	static ssize_t count = 0;

	while (true) {
		/* first read whatever is left */
		if (j < count) {
			/* pass received bytes to the packet decoder */
			//warnx("buf: %d", count);
			while (j < count) {
				if (parse_char(buf[j]) > 0) {
				fflush(stdout);
					int ret = handle_message();
					if (ret == 1){
						return 1;			// pos update
					}
					else{
						return -1;		// no known message
					}
				}

				/* in case we keep trying but only get crap from GPS */
				if (time_started + timeout * 1000 < hrt_absolute_time()) {
					return -1;
				}

				j++;
			}

			/* everything is read */
			j = count = 0;


		}

		/* then poll for new data */
		int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), timeout);

		if (ret < 0) {
			/* something went wrong when polling */
			return -1;

		} else if (ret == 0) {
			/* Timeout */
			return -1;

		} else if (ret > 0) {
			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device.  If more bytes are
				 * available, we'll go back to poll() again...
				 */
				//usleep(WEPILOT_WAIT_BEFORE_READ_USEC);
				count = ::read(_fd, buf, sizeof(buf));
			}
		}

	} //end while
}

void
WEPILOT::decode_init(void)
{
	_rx_count = 0;
	_decode_state = WEPILOT_DECODE_UNINIT;
	_rx_header_lgth = 0;
	_rx_message_lgth = 0;
	_rx_crc = 0;
	_rx_message_time_status = WEPILOT_TIME_STATUS_UNKNOWN;
	_rx_message_week = 0;
	_rx_message_ms = 0;
	_send55Flag = 0;
}


int
WEPILOT::parse_char(uint8_t b)
{
	switch (_decode_state) {
//		/* First, look for start 'A5' */
		case WEPILOT_DECODE_UNINIT:
			if (b == WEPILOT_SYNC) {
				_decode_state = WEPILOT_DECODE_GOT_SYNC;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
//				warnx("Add %02x to buffer[%d] Sync\n",b,_rx_count-1);
			} else {
//				warnx("Missed start byte: %02x\n", b);
			}
			break;
		/* Second, look at ID */
		case WEPILOT_DECODE_GOT_SYNC:
			if (b == WEPILOT_ID) {
				_decode_state = WEPILOT_DECODE_GOT_ID;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
//				warnx("Add %02x to buffer[%d] ID\n",b,_rx_count-1);
			} else {
				/* ID symbol was wrong, reset state machine */
				decode_init();
//				warnx("Missed ID byte: %d", b);
			}
			break;
		/* Third, look at Message length */
		case WEPILOT_DECODE_GOT_ID:
			if (b == WEPILOT_MsgLength) {
				_decode_state = WEPILOT_DECODE_GOT_MESSAGE_LGTH;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
//				warnx("Add %02x to buffer[%d] MsgLength\n",b,_rx_count-1);
			} else {
				/* Start symbol was wrong, reset state machine */
				decode_init();
//				warnx("Missed start byte: %02x\n", b);
				//warnx("[GPS] second start byte wrong");
		}
		break;
		/* Get the message */
		case WEPILOT_DECODE_GOT_MESSAGE_LGTH:
			if (_send55Flag) {
				if (b == SENDA5) {
					_rx_buffer[_rx_count] = 0xA5;
					_rx_count++;
//					warnx("Add %02x to buffer[%d] 0x55 0x00 -> 0xA5 was sent\n",b,_rx_count-1);
				}
				else if (b == SEND55){
					_rx_buffer[_rx_count] = 0x55;
					_rx_count++;
//					warnx("Add %02x to buffer[%d] 0x55 0x55 -> 0x55 was sent\n",b,_rx_count-1);
				}
				_send55Flag = 0;
			}

			if (b == SEND55) {
				_send55Flag = 1;
//				warnx("0x55 detected\n");
			} else {
				_rx_buffer[_rx_count] = b;
				_rx_count++;
//				warnx("Add %02x to buffer[%d]\n",b,_rx_count-1);
				if (_rx_count == CRCINDEX-1){
					_decode_state = WEPILOT_DECODE_GOT_MESSAGE;
				}
			}
			break;
		/* Get the crc */
		case WEPILOT_DECODE_GOT_MESSAGE:
			_rx_crc = b;
			_rx_buffer[_rx_count] = _rx_crc;
//			warnx("Add %02x to buffer[%d] CRC\n",b,_rx_count);
			/* compare checksum */
			_calculated_crc = calculate_block_wePilot(_rx_count+1, _rx_buffer);
			if ( _calculated_crc == _rx_crc) {
				warnx("Checksum Correct\n");
				return 1;
			} else {
				warnx("WEPILOT: Checksum wrong. calculated crc: %x, rx crc: %x", _calculated_crc, _rx_crc);
				decode_init();
				return -1;
			}
			break;
		default:
			return -1;
			break;
	}
	return 0;
}

int
WEPILOT::handle_message()
{
	int ret = 0;

//	gps_WEPILOT_bestpos_packet_t *packet_bestpos;
//	gps_WEPILOT_bestvel_packet_t *packet_bestvel;
//	gps_WEPILOT_psrdop_packet_t  *packet_psrdop;
//	gps_WEPILOT_heading_packet_t  *packet_heading;
	gps_WEPILOT_datagram *packet_raw;

	switch (_rx_message_id) {
		case WEPILOT_ID:
//				packet_bestpos = (gps_WEPILOT_bestpos_packet_t *) (&(_rx_buffer[_rx_header_lgth]));
				packet_raw = (gps_WEPILOT_datagram *) (&(_rx_buffer));

				//	Global Position
				_global_position.lat = (double)conv32bit2int32(packet_raw->latPos[3], packet_raw->latPos[2],packet_raw->latPos[1],packet_raw->latPos[0])/100000000.0L;
				_global_position.lon = (double)conv32bit2int32(packet_raw->lonPos[3], packet_raw->lonPos[2],packet_raw->lonPos[1],packet_raw->lonPos[0])/100000000.0L;
				_global_position.alt = (float)conv32bit2int32(packet_raw->altPos[3], packet_raw->altPos[2],packet_raw->altPos[1],packet_raw->altPos[0])/1000.0f;
				_global_position.vel_n = (float)conv32bit2float(packet_raw->gndVelN[3], packet_raw->gndVelN[2],packet_raw->gndVelN[1],packet_raw->gndVelN[0]);
				_global_position.vel_e = (float)conv32bit2float(packet_raw->gndVelE[3], packet_raw->gndVelE[2],packet_raw->gndVelE[1],packet_raw->gndVelE[0]);					_global_position.vel_d = (float)conv32bit2float(packet_raw->gndVelD[3], packet_raw->gndVelD[2],packet_raw->gndVelD[1],packet_raw->gndVelD[0]);

				//	Local Position
				_local_position.x = (float)conv32bit2int32(packet_raw->relPosN[3], packet_raw->relPosN[2],packet_raw->relPosN[1],packet_raw->relPosN[0])/100.0f;
				_local_position.y = (float)conv32bit2int32(packet_raw->relPosE[3], packet_raw->relPosE[2],packet_raw->relPosE[1],packet_raw->relPosE[0])/100.0f;
				_local_position.z = (float)conv32bit2int32(packet_raw->relPosD[3], packet_raw->relPosD[2],packet_raw->relPosD[1],packet_raw->relPosD[0])/100.0f;

				//	Attitude
				_attitude.roll = (float)conv16bit2int16(packet_raw->roll[1],packet_raw->roll[0])/10000.0f*180.0f/(float)M_PI;
				_attitude.pitch = (float)conv16bit2int16(packet_raw->pitch[1],packet_raw->pitch[0])/10000.0f*180.0f/(float)M_PI;
				_attitude.yaw = (float)conv16bit2int16(packet_raw->yaw[1],packet_raw->yaw[0])/10000.0f*180.0f/(float)M_PI;
				_attitude.rollspeed = (float)conv16bit2int16(packet_raw->p[1],packet_raw->p[0])/1000.0f*180.0f/(float)M_PI;
				_attitude.pitchspeed = (float)conv16bit2int16(packet_raw->q[1],packet_raw->q[0])/1000.0f*180.0f/(float)M_PI;
				_attitude.yawspeed = (float)conv16bit2int16(packet_raw->r[1],packet_raw->r[0])/1000.0f*180.0f/(float)M_PI;

				//	Bodyframe Speed
				_bodyframe_speed.vx = (float)conv16bit2int16(packet_raw->refU[1],packet_raw->refU[0])/1000.0f;
				_bodyframe_speed.vy = (float)conv16bit2int16(packet_raw->refV[1],packet_raw->refV[0])/1000.0f;
				_bodyframe_speed.vz = (float)conv16bit2int16(packet_raw->refW[1],packet_raw->refW[0])/1000.0f;
				_bodyframe_speed.yaw_sp = (float)conv16bit2int16(packet_raw->refR[1],packet_raw->refR[0])/1000.0f;

				//	wePilot
				_wePilot_info.fx = (float)conv32bit2float(packet_raw->fX[3],packet_raw->fX[2],packet_raw->fX[1],packet_raw->fX[0]);
				_wePilot_info.fy = (float)conv32bit2float(packet_raw->fY[3],packet_raw->fY[2],packet_raw->fY[1],packet_raw->fY[0]);
				_wePilot_info.fz = (float)conv32bit2float(packet_raw->fZ[3],packet_raw->fZ[2],packet_raw->fZ[1],packet_raw->fZ[0]);
				_wePilot_info.hx = conv16bit2int16(packet_raw->hx[1],packet_raw->hx[0]);
				_wePilot_info.hy = conv16bit2int16(packet_raw->hy[1],packet_raw->hy[0]);
				_wePilot_info.hz = conv16bit2int16(packet_raw->hz[1],packet_raw->hz[0]);
				_wePilot_info.rotor = conv16bit2uint16(packet_raw->rotor[1],packet_raw->rotor[0]);
				_wePilot_info.time.hours = conv8bit2uint8(packet_raw->time[3]);
				_wePilot_info.time.minutes = conv8bit2uint8(packet_raw->time[2]);
				_wePilot_info.time.seconds = (float)conv16bit2uint16(packet_raw->time[1],packet_raw->time[0])/1000.0f;
				_wePilot_info.fcsState = conv8bit2uint8(packet_raw->status[0] & 0b00001111);
				_wePilot_info.pwm_inp7 = conv8bit2uint8(packet_raw->status[0] & 0b00010000)>>4;
				_wePilot_info.gpsSat = conv8bit2uint8(packet_raw->status[1] & 0b00001111);

				//  Publish
				/* announce the Global Position if needed, just publish else */
				if (_global_position_pub > 0) orb_publish(ORB_ID(vehicle_global_position), _global_position_pub, &_global_position);
				else _global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &_global_position);
				/* announce the Local Position if needed, just publish else */
				if (_local_position_pub > 0) orb_publish(ORB_ID(vehicle_local_position), _local_position_pub, &_local_position);
				else _local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &_local_position);
				/* announce the Attitude if needed, just publish else */
				if (_attitude_pub > 0) orb_publish(ORB_ID(vehicle_attitude), _attitude_pub, &_attitude);
				else _attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &_attitude);
				/* announce the Bodyframe Speed if needed, just publish else */
//				if (_bodyframe_speed_pub > 0) orb_publish(ORB_ID(vehicle_bodyframe_speed), _bodyframe_speed_pub, &_bodyframe_speed);
//				else _bodyframe_speed_pub = orb_advertise(ORB_ID(vehicle_bodyframe_speed), &_bodyframe_speed);
				/* announce the wePilot if needed, just publish else */
//				if (_wePilot_info_pub > 0) orb_publish(ORB_ID(wePilot_info), _wePilot_info_pub, &_wePilot_info);
//				else _wePilot_info_pub = orb_advertise(ORB_ID(wePilot_info), &_wePilot_info);

				ret = 1;
				break;
		default:
					warnx("WEPILOT: Unknown message received");
					ret = -1;
					break;
	}
		decode_init();
		return ret;
}
//		case WEPILOT_BESTPOS:
//			packet_bestpos = (gps_WEPILOT_bestpos_packet_t *) (&(_rx_buffer[_rx_header_lgth]));
//
//			_gps_position->timestamp_position = hrt_absolute_time();
//			_gps_position->lat = (int32_t) (packet_bestpos->lat * 1e7);
//			_gps_position->lon = (int32_t) (packet_bestpos->lon * 1e7);
//			_gps_position->alt = (int32_t) (packet_bestpos->hgt * 1e3);
//
//
//			_gps_position->timestamp_variance = hrt_absolute_time();
//			_gps_position->c_variance_rad = (packet_bestpos->sigma_hgt) * (packet_bestpos->sigma_hgt);	// var = sigma^2
//			_gps_position->fix_type = (packet_bestpos->postype >= WEPILOT_FIXTYPE_SINGLE) ? 3 : 0;	// there is no 2d fix
//
//			_gps_position->satellites_used = packet_bestpos->sat_tracked;
//
//			if(_rx_message_time_status >= WEPILOT_TIME_STATUS_FINEADJUSTING ){
//				_gps_position->timestamp_time = hrt_absolute_time();
//				_gps_position->time_gps_usec = ((uint64_t) _rx_message_ms) * 1000;
//			}
//			_rate_count_lat_lon++;
//			_got_posllh = true;
//
//			_best_pos.time=_gps_time;
//			_best_pos.data=*packet_bestpos;
//			if(_report_best_pos>0)
//				orb_publish(ORB_ID(WEPILOT_bestpos), _report_best_pos, &_best_pos);
//			else
//				_report_best_pos=orb_advertise(ORB_ID(WEPILOT_bestpos), &_best_pos);
//
//			ret = 1;
//			break;
//		case WEPILOT_BESTVEL:
//			packet_bestvel = (gps_WEPILOT_bestvel_packet_t *) (&(_rx_buffer[_rx_header_lgth]));
//
//			_gps_position->timestamp_velocity = hrt_absolute_time();
//			_gps_position->vel_m_s = (float) packet_bestvel->hor_spd;
//			_gps_position->vel_n_m_s = ((float) packet_bestvel->hor_spd) * cosf((	(float) packet_bestvel->trk_gnd) * M_DEG_TO_RAD_F);
//			_gps_position->vel_e_m_s = ((float) packet_bestvel->hor_spd) * sinf(((float) packet_bestvel->trk_gnd) * M_DEG_TO_RAD_F);
//			_gps_position->vel_d_m_s = -((float) packet_bestvel->vert_spd);
//			// configure receiver with "magvar correction 0 0" (default) to be sure that course over ground = heading
//			_gps_position->cog_rad = (((float) packet_bestvel->trk_gnd) < 180.0f) ? (( (float) packet_bestvel->trk_gnd) * M_DEG_TO_RAD_F) : ((( (float) packet_bestvel->trk_gnd) - 360.0f) * M_DEG_TO_RAD_F);	// 0-360 to -pi..+pi
//			_gps_position->vel_ned_valid = (packet_bestvel->veltype >= WEPILOT_FIXTYPE_SINGLE) ? true : false;
//			_gps_position->s_variance_m_s = 0;									//does not exist in WEPILOT protocol
//
//			if(_rx_message_time_status >= WEPILOT_TIME_STATUS_FINEADJUSTING ){
//				_gps_position->timestamp_time = hrt_absolute_time();
//				_gps_position->time_gps_usec = _rx_message_ms * 1000;
//			}
//
//			_rate_count_vel++;
//			_got_velned = true;
//
//			_best_vel.time=_gps_time;
//			_best_vel.data=*packet_bestvel;
//			if(_report_best_vel>0)
//				orb_publish(ORB_ID(WEPILOT_bestvel), _report_best_vel, &_best_vel);
//			else
//				_report_best_vel=orb_advertise(ORB_ID(WEPILOT_bestvel), &_best_vel);
//
//
//			ret = 2;
//			break;
//		case WEPILOT_PSRDOP:
//			packet_psrdop = (gps_WEPILOT_psrdop_packet_t *) (&(_rx_buffer[_rx_header_lgth]));
//
//			_gps_position->eph = packet_psrdop->hdop;
//			_gps_position->epv = sqrt((packet_psrdop->pdop * packet_psrdop->pdop) - (packet_psrdop->hdop * packet_psrdop->hdop));
//
//			ret = 3;	// dirty, 0 means no message handeled.
//			break;
//		case WEPILOT_HEADING:
//			packet_heading = (gps_WEPILOT_heading_packet_t *) (&(_rx_buffer[_rx_header_lgth]));
//
//			_gps_heading->hdg=packet_heading->heading;
//			_gps_heading->hdg_deviation=packet_heading->hdg_std_dev;
//			_gps_heading->ptch=packet_heading->pitch;
//			_gps_heading->ptch_deviation=packet_heading->ptch_std_dev;
//
//			if(_rx_message_time_status >= WEPILOT_TIME_STATUS_FINEADJUSTING ){
//				_gps_heading->timestamp_time = hrt_absolute_time();
//				_gps_heading->time_gps_usec = _rx_message_ms * 1000;
//			}
//
//			_heading.time=_gps_time;
//			_heading.data=*packet_heading;
//			if(_report_heading>0)
//				orb_publish(ORB_ID(WEPILOT_heading), _report_heading, &_heading);
//			else
//				_report_heading=orb_advertise(ORB_ID(WEPILOT_heading), &_heading);
//
//			ret = 4;
//			break;
//			}


uint8_t
WEPILOT::calculate_block_wePilot(unsigned long message_lgth, uint8_t *data)
{
	register unsigned long i;
	uint16_t sum = 0x00;
	uint8_t crc = 0x00;

	for (i=3; i<(message_lgth-1); i++) // payload starts after lengthMessage-byte and ends before crc-byte
	{
		sum+= data[i]; // sum of all payload bytes
	}
	crc = (uint8_t)(sum % (uint16_t)256);

	return crc;
}

unsigned long
WEPILOT::calculate_block_crc32(unsigned long message_lgth, unsigned char *data)
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while ( message_lgth-- != 0 )
	{
		ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value( ((int) ulCRC ^ *data++ ) & 0xff );
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return ulCRC;
}

unsigned long
WEPILOT::CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;
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
uint8_t
WEPILOT::conv8bit2uint8(uint8_t msb){
	return ((uint8_t)msb);
}
int8_t
WEPILOT::conv8bit2int8(uint8_t msb){
	return ((int8_t)msb);
}
uint16_t
WEPILOT::conv16bit2uint16(uint8_t msb, uint8_t lsb){
	return ((uint16_t)((uint16_t)msb<<8) | ((uint16_t)lsb));
}
int16_t
WEPILOT::conv16bit2int16(uint8_t msb, uint8_t lsb){
	return ((int16_t)((int16_t)msb<<8) | ((int16_t)lsb));
}
uint32_t
WEPILOT::conv32bit2uint32(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb){
	return ((uint32_t)((uint32_t)msb<<24) | ((uint32_t)mid2b<<16) | ((uint32_t)mid1b<<8) | ((uint32_t)lsb));
}
int32_t
WEPILOT::conv32bit2int32(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb){
	return ((int32_t)((int32_t)msb<<24) | ((int32_t)mid2b<<16) | ((int32_t)mid1b<<8) | ((int32_t)lsb));
}
float
WEPILOT::conv32bit2float(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb){
	union{
		char bytes[4];
		float fp;
	} un;
	un.bytes[0] = lsb;
	un.bytes[1] = mid1b;
	un.bytes[2] = mid2b;
	un.bytes[3] = msb;

	return un.fp;
}



