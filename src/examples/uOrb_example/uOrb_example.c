/*
 * uOrb_example.c
 *
 *  Created on: 19.08.2015
 *      Author: tetresch
 */


#include "publisher.h"
#include "subscriber.h"
#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

//#include <uORB/uORB.h>


__EXPORT int uOrb_example_main(int argc, char *argv[]);

int uOrb_example_main(int argc, char *argv[])
{
	printf("Hello Sky!\n");
	publishInit();
	subscribeInit();
	update_topic();
	check_topic();

	return 0;

}


