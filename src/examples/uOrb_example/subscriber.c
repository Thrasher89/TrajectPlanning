/*
 * subscriber.c
 *
 *  Created on: 19.08.2015
 *      Author: tetresch
 */

#include <stdio.h>
#include <stdint.h>
#include "topic.h"
#include "subscriber.h"
#include <uORB/uORB.h>


/* file handle that will be used for subscribing */
static int topic_handle;


void subscribeInit()
{
	/* subscribe to the topic */
	topic_handle = orb_subscribe(ORB_ID(random_integer));
}

void check_topic()
{
	bool updated;
	struct random_integer_data rd;

	/* check to see whether the topic has updated since the last time we read it */
	orb_check(topic_handle, &updated);

	if (updated) {
		/* make a local copy of the updated data structure */
		orb_copy(ORB_ID(random_integer), topic_handle, &rd);
		printf("Random integer is now %d\n", rd.r);
	}
}
