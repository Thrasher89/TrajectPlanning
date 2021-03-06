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

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "stim300.h"
#include "serial.h"


#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h> //including timestamp funktion hrt_absolute_time()
#include <uORB/uORB.h>




using namespace stim300;


/*
   *crc_table for crc polynom 0x04c11db7
   *
   */
static uint32_t crc_table[256] = {
0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b,
0x1a864db2, 0x1e475005, 0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd, 0x4c11db70, 0x48d0c6c7,
0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3,
0x709f7b7a, 0x745e66cd, 0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5, 0xbe2b5b58, 0xbaea46ef,
0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb,
0xceb42022, 0xca753d95, 0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d, 0x34867077, 0x30476dc0,
0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4,
0x0808d07d, 0x0cc9cdca, 0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02, 0x5e9f46bf, 0x5a5e5b08,
0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc,
0xb6238b25, 0xb2e29692, 0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a, 0xe0b41de7, 0xe4750050,
0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34,
0xdc3abded, 0xd8fba05a, 0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb, 0x4f040d56, 0x4bc510e1,
0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5,
0x3f9b762c, 0x3b5a6b9b, 0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623, 0xf12f560e, 0xf5ee4bb9,
0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd,
0xcda1f604, 0xc960ebb3, 0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b, 0x9b3660c6, 0x9ff77d71,
0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2,
0x470cdd2b, 0x43cdc09c, 0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24, 0x119b4be9, 0x155a565e,
0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a,
0x2d15ebe3, 0x29d4f654, 0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c, 0xe3a1cbc1, 0xe760d676,
0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662,
0x933eb0bb, 0x97ffad0c, 0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4};

//uint32_t stim300_crc32 (uint8_t *data, int len, int dummyBytes);
uint32_t
Stim300::stim300_crc32(uint8_t *data, int len, int dummyBytes)
{
	register int i;
	uint32_t crc = 0xffffffff;

	for (i=0; i<len; i++)
		crc = (crc << 8) ^ crc_table[((crc >> 24) ^ *data++) & 0xff];
	for (i=0; i<dummyBytes; i++)
		crc = (crc << 8) ^ crc_table[((crc >> 24) ^ 0x00) & 0xff];

	return crc;
}
uint8_t
Stim300::wePilot_crc(uint8_t *data, int len)
{
	register int i;
	uint16_t sum = 0x00;
	uint8_t crc = 0x00;

	for (i=2; i<(len-1); i++) // payload starts after lengthMessage-byte and ends before crc-byte
	{
		sum = data[i] + sum; // sum of all payload bytes
	}
	crc = (uint8_t)(sum % (uint16_t)256);

	return crc;
}




Stim300*
Stim300::Instance()
{
	static Stim300* singelton=NULL;
	return singelton ? singelton : (singelton = new Stim300());
}
Stim300::Stim300()
{
	printf("\n\ncreating Class\n\n");
	Serial::init("/dev/ttyS2",57600); //"/dev/ttyS2",921600

	/* publications */
	_local_position_pub = -1;

};
Stim300::~Stim300(){
}
uint8_t*
Stim300::getDatagramm(dataGramms type){
	datagrammReadCnt++;
	getData(dataBuffer,getDatagrammSize(type));
	/**
	  * check if id is correct and has CR LF
	  */
	if(dataBuffer[0]!=(uint8_t)type){
		datagrammReadErrCnt++;
		sync2CR_LF();
		return NULL;
	}
	if(dataBuffer[getDatagrammSize(type)-2]!=0x0d || dataBuffer[getDatagrammSize(type)-1]!=0x0a){
		datagrammReadErrCnt++;
		sync2CR_LF();
		return NULL;
	}
//	//switch(type){
//	//	case NORMAL_MODE_93:
	if (type == NORMAL_MODE_93){
		      struct s_datagram_normalMode_93* s = reinterpret_cast<struct s_datagram_normalMode_93*>(dataBuffer);
		      uint8_t toBeCrc[4] =  {s->crc[3] , s->crc[2] ,s->crc[1] ,s->crc[0]}; //endian correction
		      if(*((uint32_t*)toBeCrc) != stim300_crc32(dataBufferWePilot, getDatagrammSize(type)-2-4, 2)){ //2dummy 0x00's
			      datagrammCRCErrCnt++;
			      return NULL;
		      }
	}


	return dataBuffer;
}
uint8_t*
Stim300::getDatagrammWePilot(dataGramms type){
	datagrammReadCnt++;
	uint16_t checkSum;
	uint8_t toBeCrc;

	checkSum = getDataWePilot(dataBufferWePilot,getDatagrammSize(type));
	/**
	  * check if id is correct
	  */
	if(dataBufferWePilot[0]!=(uint8_t)type){
		datagrammReadErrCnt++;
		sync2_0xA5();
		return NULL;
	}
	/**
		  * check crc id is correct
		  */
	if (type == wePilot){
//		toBeCrc =  wePilot_crc(dataBufferWePilot, getDatagrammSize(type));
		toBeCrc =  (uint8_t)(checkSum % (uint16_t)256);

		      if(dataBufferWePilot[getDatagrammSize(type)-1] != toBeCrc){
			      datagrammCRCErrCnt++;
			      printf("checksum error crcCalc: %02x instead of: %02x\n",toBeCrc,dataBufferWePilot[getDatagrammSize(type)-1]);
			      return NULL;
		      }
	}

	return dataBufferWePilot;
}
uint8_t*
Stim300::getDatagrammWePilotFast(dataGramms type){
	datagrammReadCnt++;
	uint8_t crc;
//	uint8_t toBeCrc;
//	hrt_abstime timeStart; //typedef uint64_t hrt_abstime; src/drivers/stm32/drv_hrt.c
//	int Start2DatareadTime;
//
//	timeStart = hrt_absolute_time(); // Start time
//	checkSum =
	crc = getDataWePilotFast(dataBufferWePilot,getDatagrammSize(type));
//	Start2DatareadTime  = (int)(hrt_absolute_time()-timeStart); // Stop time
//	printf("Start2DatareadTime: %u,\n",Start2DatareadTime);
	/**
	  * check if id is correct
	  */
	if(dataBufferWePilot[0]!=(uint8_t)type){
		datagrammReadErrCnt++;
		sync2_0xA5();
		return NULL;
	}
	/**
		  * check crc id is correct
		  */

	if(dataBufferWePilot[getDatagrammSize(type)-1] != crc){
		datagrammCRCErrCnt++;
		printf("checksum error crcCalc: %02x instead of: %02x\n",crc,dataBufferWePilot[getDatagrammSize(type)-1]);
//		return NULL;
	}

	return dataBufferWePilot;
}
unsigned
Stim300::getDatagrammSize(dataGramms type)
{
	switch(type){
		case PART_NO:
			return sizeof(struct s_datagram_partNo);
		case SERIAL_NO:
			return sizeof(struct s_datagram_serialNo);
		case CONFIG:
			return sizeof(struct s_datagram_config);
		case NORMAL_MODE_93:
			return sizeof(struct s_datagram_normalMode_93);
		case wePilot:
			return sizeof(struct s_datagram_wePilot);
		default:
			printf("Stim300::getStructSize type not found\n");
			return 0;
	}
	return 0;
}

void 
Stim300::putCmd(char* cmd)
{
	if(sizeof(char) != sizeof(uint8_t)){
		printf("Stim300:putCmd require sizeof(char)=8\n");
		return;
	}
	putData((uint8_t*)cmd,strlen(cmd));


}
void
Stim300::startUp(){
	state=startingUp;
	serialOpen();

	usleep(100);
//	if(!isDataAvailable()){
//		printf("Stim300 not connected or power up\n");
//		while(!isDataAvailable()){
//			sleep(1);
//		}
//	}
	/**
	  * Serial data available here
	  * check of Stim300
	  */


	for(int i=3; i>0; i--){
		printf("\rStarting up Stim300 in %d",i);
		fflush(stdout);
		sleep(1);
	}
	printf("\rStarting up");
	serialOpen(true);

	/**
	  *Dont know why but I have to send SERVICEMOD 3times to work
	  */
	putCmd((char*)"SERVICEMODE\r");
	putCmd((char*)"SERVICEMODE\r");
	putCmd((char*)"SERVICEMODE\r");
	//sync();
	/**
	  * No sync or flush function in Nuttix yet
	  * so close and open port
	   */
	serialOpen(true);
	
	/**
	  *Print sensor infos
	  */
	putCmd((char*)"i\r");
	usleep(50);
	while(isDataAvailable())
		printAscii(1);
	putCmd((char*)"x N\r");
	putCmd((char*)"x N\r");

	// start manipulation
	printf("start reading\n");
	::fflush(NULL);
	usleep(1000);
	printf("start reading now\n");


// ##############
// ##   START  ##
// ##############

	/*
	* do subscriptions
	*/

	_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/*
	* do advertisements
	*/
	struct s_datagram_wePilot* raw;
	memset(&raw, 0, sizeof(raw));
	/* advertise the sensor_combined topic and make the initial publication */
	_global_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &raw);
	_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &raw);
	_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &raw);


	hrt_abstime timeStart; //typedef uint64_t hrt_abstime; src/drivers/stm32/drv_hrt.c
//	int Start2DatareadTime;
//	int Start2ConvertionTime;
//	int Start2PublishTime;
	int timeTotal;


	uint8_t* receivedData;
	int cnt = 0;

	timeStart = hrt_absolute_time(); // Start time
	while(cnt<100){
	sync2_0xA5();


	receivedData = getDatagrammWePilot(wePilot);
//	receivedData = getDatagrammWePilotFast(wePilot);

//	Start2DatareadTime  = (int)(hrt_absolute_time()-timeStart); // Stop time

	raw = reinterpret_cast<struct s_datagram_wePilot*>(receivedData);

//	Global Position
	_global_position.lat = (double)conv32bit2int32(raw->latPos[3], raw->latPos[2],raw->latPos[1],raw->latPos[0])/100000000.0L;
	_global_position.lon = (double)conv32bit2int32(raw->lonPos[3], raw->lonPos[2],raw->lonPos[1],raw->lonPos[0])/100000000.0L;
	_global_position.alt = (float)conv32bit2int32(raw->altPos[3], raw->altPos[2],raw->altPos[1],raw->altPos[0])/1000.0f;
	_global_position.vel_n = (float)conv32bit2float(raw->gndVelN[3], raw->gndVelN[2],raw->gndVelN[1],raw->gndVelN[0]);
	_global_position.vel_e = (float)conv32bit2float(raw->gndVelE[3], raw->gndVelE[2],raw->gndVelE[1],raw->gndVelE[0]);
	_global_position.vel_d = (float)conv32bit2float(raw->gndVelD[3], raw->gndVelD[2],raw->gndVelD[1],raw->gndVelD[0]);

//	Local Position
	_local_position.x = (float)conv32bit2int32(raw->relPosN[3], raw->relPosN[2],raw->relPosN[1],raw->relPosN[0])/100.0f;
	_local_position.y = (float)conv32bit2int32(raw->relPosE[3], raw->relPosE[2],raw->relPosE[1],raw->relPosE[0])/100.0f;
	_local_position.z = (float)conv32bit2int32(raw->relPosD[3], raw->relPosD[2],raw->relPosD[1],raw->relPosD[0])/100.0f;

//	Attitude
	_attitude.roll = (float)conv16bit2int16(raw->roll[1],raw->roll[0])/10000.0f*180.0f/(float)M_PI;
	_attitude.pitch = (float)conv16bit2int16(raw->pitch[1],raw->pitch[0])/10000.0f*180.0f/(float)M_PI;
	_attitude.yaw = (float)conv16bit2int16(raw->yaw[1],raw->yaw[0])/10000.0f*180.0f/(float)M_PI;
	_attitude.rollspeed = (float)conv16bit2int16(raw->p[1],raw->p[0])/1000.0f*180.0f/(float)M_PI;
	_attitude.pitchspeed = (float)conv16bit2int16(raw->q[1],raw->q[0])/1000.0f*180.0f/(float)M_PI;
	_attitude.yawspeed = (float)conv16bit2int16(raw->r[1],raw->r[0])/1000.0f*180.0f/(float)M_PI;

//	Bodyframe Speed
	_bodyframe_speed.vx = (float)conv16bit2int16(raw->refU[1],raw->refU[0])/1000.0f;
	_bodyframe_speed.vy = (float)conv16bit2int16(raw->refV[1],raw->refV[0])/1000.0f;
	_bodyframe_speed.vz = (float)conv16bit2int16(raw->refW[1],raw->refW[0])/1000.0f;
	_bodyframe_speed.yaw_sp = (float)conv16bit2int16(raw->refR[1],raw->refR[0])/1000.0f;

//	wePilot
	_wePilot.fx = (float)conv32bit2float(raw->fX[3],raw->fX[2],raw->fX[1],raw->fX[0]);
	_wePilot.fy = (float)conv32bit2float(raw->fY[3],raw->fY[2],raw->fY[1],raw->fY[0]);
	_wePilot.fz = (float)conv32bit2float(raw->fZ[3],raw->fZ[2],raw->fZ[1],raw->fZ[0]);
	_wePilot.hx = conv16bit2int16(raw->hx[1],raw->hx[0]);
	_wePilot.hy = conv16bit2int16(raw->hy[1],raw->hy[0]);
	_wePilot.hz = conv16bit2int16(raw->hz[1],raw->hz[0]);
	_wePilot.rotor = conv16bit2uint16(raw->rotor[1],raw->rotor[0]);
	_wePilot.time.hours = conv8bit2uint8(raw->time[3]);
	_wePilot.time.minutes = conv8bit2uint8(raw->time[2]);
	_wePilot.time.seconds = (float)conv16bit2uint16(raw->time[1],raw->time[0])/1000.0f;
	_wePilot.fcsState = conv8bit2uint8(raw->status[0] & 0b00001111);
	_wePilot.pwm_inp7 = conv8bit2uint8(raw->status[0] & 0b00010000)>>4;
	_wePilot.gpsSat = conv8bit2uint8(raw->status[1] & 0b00001111);


// TBD: ref-Werte, time, status...

//	Start2ConvertionTime  = (int)(hrt_absolute_time()-timeStart); // Stop time

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
	if (_bodyframe_speed_pub > 0) orb_publish(ORB_ID(vehicle_bodyframe_speed), _bodyframe_speed_pub, &_bodyframe_speed);
	else _bodyframe_speed_pub = orb_advertise(ORB_ID(vehicle_bodyframe_speed), &_bodyframe_speed);
	/* announce the wePilot if needed, just publish else */
	if (_wePilot_pub > 0) orb_publish(ORB_ID(wePilot), _wePilot_pub, &_wePilot);
	else _wePilot_pub = orb_advertise(ORB_ID(wePilot), &_wePilot);

//	Start2PublishTime  = (int)(hrt_absolute_time()-timeStart); // Stop time

//  Subscribe

	bool updated;
//	Check update Global Position
	orb_check(_global_position_sub, &updated);
	if (updated) orb_copy(ORB_ID(vehicle_global_position), _global_position_sub, &_global_position);
	else printf("Global Position not subscribed!\n");
//	Check update Local Position
	orb_check(_local_position_sub, &updated);
	if (updated) orb_copy(ORB_ID(vehicle_local_position), _local_position_sub, &_local_position);
	else printf("Local Position not subscribed!\n");
//	Check update Attitude
	orb_check(_attitude_sub, &updated);
	if (updated) orb_copy(ORB_ID(vehicle_attitude), _attitude_sub, &_attitude);
	else printf("Attitude not subscribed!\n");
//	Check update Bodyframe Speed
	orb_check(_bodyframe_speed_sub, &updated);
	if (updated) orb_copy(ORB_ID(vehicle_bodyframe_speed), _bodyframe_speed_sub, &_bodyframe_speed);
	else printf("Bodyframe Speed not subscribed!\n");
	//	Check update wePilot
	orb_check(_wePilot_sub, &updated);
	if (updated) orb_copy(ORB_ID(wePilot), _wePilot_sub, &_wePilot);
	else printf("wePilot not subscribed!\n");

	//	Print all datas

//
		printf("Message: %d\n",cnt);
		printf("Global Position: lat: %4.8f, long: %4.8f, alt: %4.1f\n", (double)_global_position.lat, (double)_global_position.lon, (double)_global_position.alt);
		printf("Global Position: vel_n: %4.2f, vel_e: %4.2f, vel_d: %4.2f\n", (double)_global_position.vel_n, (double)_global_position.vel_e, (double)_global_position.vel_d);
		printf("Local Position: x: %4.2f, y: %4.2f, z: %4.2f\n", (double)_local_position.x, (double)_local_position.y, (double)_local_position.z);
		printf("Attitude: roll: %4.8f, pitch: %4.8f, yaw: %4.8f\n", (double)_attitude.roll, (double)_attitude.pitch, (double)_attitude.yaw);
		printf("Attitude: rollspeed: %4.8f, pitchspeed: %4.8f, yawspeed: %4.8f\n", (double)_attitude.rollspeed, (double)_attitude.pitchspeed, (double)_attitude.yawspeed);
		printf("Bodyframe: vx: %4.8f, vy: %4.8f, vz: %4.8f\n", (double)_bodyframe_speed.vx, (double)_bodyframe_speed.vy, (double)_bodyframe_speed.vz, (double)_bodyframe_speed.yaw_sp);
		printf("wePilot: fx: %4.8f, fy: %4.8f, fz: %4.8f\n", (double)_wePilot.fx, (double)_wePilot.fy, (double)_wePilot.fz);
		printf("wePilot: hx: %d, hy: %d, hz: %d\n", _wePilot.hx, _wePilot.hy, _wePilot.hz);
		printf("wePilot: Hours: %u, Minutes: %u, Seconds: %4.8f\n", _wePilot.time.hours, _wePilot.time.minutes, (double)_wePilot.time.seconds);
		printf("wePilot: Rotor: %u, FCS State: %u, PWM: %u, GPS Sat: %u\n", _wePilot.rotor, _wePilot.fcsState, _wePilot.pwm_inp7, _wePilot.gpsSat);
		printf("\n");

//		printf("Cnt: %d, Start2DatareadTime: %u, Start2ConvertionTime: %u, Start2PublishTime: %u, timeTotal: %u\n",cnt,Start2DatareadTime, Start2ConvertionTime, Start2PublishTime, timeTotal);


	cnt++;
	}
	timeTotal  = (int)(hrt_absolute_time()-timeStart); // Stop time
	printf("Cnt: %d, , timeTotal: %u\n",cnt, timeTotal);

	/**
	  *We have to read the answer value that the sensor goes back to normalmode
	  */
	uint8_t temp[101];
	getData(temp,100);
	serialClose();
	usleep(200);
	state=idle;

task_create("Stim300",
sched_get_priority_max(SCHED_RR),
2048,
(main_t)readThreadTrampolin, (char* const*)NULL);

}
void
Stim300::Start(){
	Stim300* s = Instance();
	if(s->state>idle)
		printf("Sensor already started");
	else
		Stim300::Instance()->startUp();
	/**
	  *make shure we are initialisasied
	  */
}
char* 
Stim300::getState(){
	Stim300* s = Instance();
	switch(s->state){
		case uninitialized:
			return (char*)"Uninitialized";
		case startingUp:
			return (char*)"Starting up";
		case idle:
			return (char*)"Sensor available but Idle";
		case streaming:
			return (char*)"Streaming";
	
	}
	return (char*)"State not found";
}
int
Stim300::readThreadTrampolin(const char** data)
{
	Stim300::Instance()->readThread();
	return NULL;
}
void
Stim300::readThread()
{
	printf("in readThread\n");
	printf("start stream reading\n");
	serialOpen();
	out.init("/dev/ttyACM0",921600);
	out.serialOpen(true);
#ifdef STREAM_MPU600
	board_accel_init();
	int _accel_sub= orb_subscribe(ORB_ID(sensor_accel0));
	int _gyro_sub= orb_subscribe(ORB_ID(sensor_gyro0));
	bool accel_updated;
	struct accel_report accel_report;
	struct gyro_report gyro_report;
#endif
//	bool gps_updated;
//	int _gps_heading_sub = orb_subscribe(ORB_ID(novatel_heading));
//	int _gps_position_sub = orb_subscribe(ORB_ID(novatel_bestpos));
//	int _gps_velocity_sub = orb_subscribe(ORB_ID(novatel_bestvel));


		/*
		float ax,ay,az;
		float gx,gy,gz;
		ax=ay=az=gx=gy=gz=0;
		unsigned downsample=1;
		*/
		uint8_t outBuffer[64];

		
		::fflush(NULL);
		//int n=0;
		//::read(outfd,&n,1);
		printf("start streaming\n");
		sync2CR_LF();


	//for(int k=0;k<2000*60;k++){
		state=streaming;

		uint8_t* data;
		struct s_datagram_normalMode_93* pack;
		hrt_abstime time; //typedef uint64_t hrt_abstime; src/drivers/stm32/drv_hrt.c
		int i=0;
	while(1){
		i=0;


		data = getDatagramm(NORMAL_MODE_93);
		if(data==NULL)
			continue;
		pack = reinterpret_cast<struct s_datagram_normalMode_93*>(data);
		time=hrt_absolute_time();





		/*
		if(n++==downsample){
			dprintf(outfd,"%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n",
					(double)ax/downsample,(double)ay/downsample,(double)az/downsample,
					(double)gx/downsample,(double)gy/downsample,(double)gz/downsample);
			n=0;
			ax=ay=az=gx=gy=gz=0;
		}
		else{
			ax+=conv24bit2float(pack->accX[0],pack->accX[1],pack->accX[2],524288);
			ay+=conv24bit2float(pack->accY[0],pack->accY[1],pack->accY[2],524288);
			az+=conv24bit2float(pack->accZ[0],pack->accZ[1],pack->accZ[2],524288);
			gx+=conv24bit2float(pack->gyroX[0],pack->gyroX[1],pack->gyroX[2],2097152);
			gy+=conv24bit2float(pack->gyroY[0],pack->gyroY[1],pack->gyroY[2],2097152);
			gz+=conv24bit2float(pack->gyroZ[0],pack->gyroZ[1],pack->gyroZ[2],2097152);
		}
		*/

		outBuffer[i++]='s';
		memcpy(&outBuffer[i],&time,sizeof(time));
		i=i+sizeof(time); //i=8
		outBuffer[i++]=pack->cnt;
		outBuffer[i++]=pack->accX[2];
		outBuffer[i++]=pack->accX[1];
		outBuffer[i++]=pack->accX[0];
		outBuffer[i++]=pack->accY[2];
		outBuffer[i++]=pack->accY[1];
		outBuffer[i++]=pack->accY[0];
		outBuffer[i++]=pack->accZ[2];
		outBuffer[i++]=pack->accZ[1];
		outBuffer[i++]=pack->accZ[0];
		outBuffer[i++]=pack->gyroX[2];
		outBuffer[i++]=pack->gyroX[1];
		outBuffer[i++]=pack->gyroX[0];
		outBuffer[i++]=pack->gyroY[2];
		outBuffer[i++]=pack->gyroY[1];
		outBuffer[i++]=pack->gyroY[0];
		outBuffer[i++]=pack->gyroZ[2];
		outBuffer[i++]=pack->gyroZ[1];
		outBuffer[i++]=pack->gyroZ[0];
		outBuffer[i++]=pack->gyroStat;
		outBuffer[i++]=pack->accStat;
		outBuffer[i++]=pack->incStat;
		outBuffer[i++]='\r';
		outBuffer[i++]='\n';
		out.putData(outBuffer,i);
			
#ifdef STREAM_MPU600
	orb_check(_accel_sub, &accel_updated);
	if (accel_updated) {
		/**
		  *As accel and gyro rom mpu6000 are updated in the same function. I intented gyro is also updated and has same timestamp 
		  */
		orb_copy(ORB_ID(sensor_accel0), _accel_sub, &accel_report);
		orb_copy(ORB_ID(sensor_gyro0), _gyro_sub, &gyro_report);
		i=0;
		outBuffer[i++]='m';
		memcpy(&outBuffer[i],&(accel_report.timestamp),sizeof(accel_report.timestamp));
		i=i+sizeof(accel_report.timestamp); //i=8
		memcpy(&outBuffer[i],&(accel_report.x_raw),sizeof(accel_report.x_raw));
		i=i+sizeof(accel_report.x_raw); //i=2
		memcpy(&outBuffer[i],&(accel_report.y_raw),sizeof(accel_report.y_raw));
		i=i+sizeof(accel_report.y_raw); //i=2
		memcpy(&outBuffer[i],&(accel_report.z_raw),sizeof(accel_report.z_raw));
		i=i+sizeof(accel_report.z_raw); //i=2
		memcpy(&outBuffer[i],&(gyro_report.x_raw),sizeof(gyro_report.x_raw));
		i=i+sizeof(gyro_report.x_raw); //i=2
		memcpy(&outBuffer[i],&(gyro_report.y_raw),sizeof(gyro_report.y_raw));
		i=i+sizeof(gyro_report.y_raw); //i=2
		memcpy(&outBuffer[i],&(gyro_report.z_raw),sizeof(gyro_report.z_raw));
		i=i+sizeof(accel_report.z_raw); //i=2
		outBuffer[i++]='\r';
		outBuffer[i++]='\n';
		out.putData(outBuffer,i);

	}
#endif
		/*GPS Position*/
	/*		orb_check(_gps_position_sub, &gps_updated);
		if (gps_updated) {
			struct novatel_bestpos_s pos;
			orb_copy(ORB_ID(novatel_bestpos), _gps_position_sub, &pos);
			i=0;
			outBuffer[i++]='p';
			out.putData(outBuffer,i);
			out.putData((uint8_t *)&pos,sizeof(pos));
			i=0;
			outBuffer[i++]='\r';
			outBuffer[i++]='\n';
			out.putData(outBuffer,i);
		}*/

		/*GPS Velocity*/
	/*		orb_check(_gps_velocity_sub, &gps_updated);
		if (gps_updated) {
			struct novatel_bestvel_s vel;
			orb_copy(ORB_ID(novatel_bestvel), _gps_velocity_sub, &vel);
			i=0;
			outBuffer[i++]='v';
			out.putData(outBuffer,i);
			out.putData((uint8_t *)&vel,sizeof(vel));
			i=0;
			outBuffer[i++]='\r';
			outBuffer[i++]='\n';
			out.putData(outBuffer,i);
		}*/

		/*GPS Heading*/
	/*		orb_check(_gps_heading_sub, &gps_updated);
		if (gps_updated) {
			struct novatel_heading_s heading;
			orb_copy(ORB_ID(novatel_heading), _gps_heading_sub, &heading);
			i=0;
			outBuffer[i++]='h';
			out.putData(outBuffer,i);
			out.putData((uint8_t *)&heading,sizeof(heading));
			i=0;
			outBuffer[i++]='\r';
			outBuffer[i++]='\n';
			out.putData(outBuffer,i);
		}*/

	}
	out.serialClose();
}
#ifdef STREAM_MPU600 
void
Stim300::board_accel_init(void){
	int accel;
	accel  = ::open(ACCEL_DEVICE_PATH, 0);
	 if (accel < 0) { 
		 printf("%s: ", ACCEL_DEVICE_PATH);
		 printf("FATAL: no onboard accelerometer found. Check if mpu6000 is started");
		 /* set the accel internal sampling rate up to at leat 1000Hz */
		 ::ioctl(accel, ACCELIOCSSAMPLERATE, 1000);

		 /* set the driver to poll at 1000Hz */
		 ::ioctl(accel, SENSORIOCSPOLLRATE, 1000);
	 } else {
		 ::close(accel);
	 }



}
#endif



void
Stim300::getInfo()
{
	printf("\nState: %s\n",getState());
	printf("\nSensorPort pack Read:%d, Sync Errors:%d, CRC Errors: %d\n\n",datagrammReadCnt,datagrammReadErrCnt,datagrammCRCErrCnt);
}

uint8_t
Stim300::conv8bit2uint8(uint8_t msb){
	return ((uint8_t)msb);
}
int8_t
Stim300::conv8bit2int8(uint8_t msb){
	return ((int8_t)msb);
}
uint16_t
Stim300::conv16bit2uint16(uint8_t msb, uint8_t lsb){
	return ((uint16_t)((uint16_t)msb<<8) | ((uint16_t)lsb));
}
int16_t
Stim300::conv16bit2int16(uint8_t msb, uint8_t lsb){
	return ((int16_t)((int16_t)msb<<8) | ((int16_t)lsb));
}
float
Stim300::conv24bit2float(uint8_t msb, uint8_t midb, uint8_t lsb, unsigned int divideBy){
	if((msb & 0b10000000) >0 )
		return (float)((int32_t)((int32_t)0xFF << 24 |((uint32_t)msb<<16) | ((uint32_t)midb<<8) | ((uint32_t)lsb)))/(float)divideBy;
	else
		return (float)(((uint32_t)msb<<16) | ((uint32_t)midb<<8) | ((uint32_t)lsb))/(float)divideBy;
}
uint32_t
Stim300::conv32bit2uint32(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb){
	return ((uint32_t)((uint32_t)msb<<24) | ((uint32_t)mid2b<<16) | ((uint32_t)mid1b<<8) | ((uint32_t)lsb));
}
int32_t
Stim300::conv32bit2int32(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb){
	return ((int32_t)((int32_t)msb<<24) | ((int32_t)mid2b<<16) | ((int32_t)mid1b<<8) | ((int32_t)lsb));
}
float
Stim300::conv32bit2float(uint8_t msb, uint8_t mid2b, uint8_t mid1b, uint8_t lsb){
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

int32_t 
Stim300::get32bit(uint8_t msb, uint8_t midb, uint8_t lsb){
	if((msb & 0b10000000) >0 )
		return (int32_t)((int32_t)0xFF << 24 | ((int32_t)msb<<16) | ((int32_t)midb<<8) | ((int32_t)lsb));
	else
		return (int32_t)((int32_t)msb<<16) | ((int32_t)midb<<8) | ((int32_t)lsb);
}


extern "C" { __EXPORT int stim300_main (int argc, char **argv); }
int stim300_main (int argc, char **argv){

	if(!strcmp(argv[1],"info") || !strcmp(argv[1],"print")){
		printf("float:%d\n",sizeof(float));
		Stim300::Instance()->getInfo();
		return 0;
	}

	
	Stim300::Start();
	return 0;
}


