/*
 * publisher.c
 *
 *  Created on: 19.08.2015
 *      Author: tetresch
 */

#include "topic.h"
#include "publisher.h"
#include <stdint.h>
#include <stdlib.h>
#include <uORB/uORB.h>


/* create topic metadata */
ORB_DEFINE(random_integer, struct random_integer_data);

/* file handle that will be used for publishing */
static int topic_handle;

void publishInit()
{
	/* generate the initial data for first publication */
	struct random_integer_data rd = { .r = rand() };

	/* advertise the topic and make the initial publication */
	topic_handle = orb_advertise(ORB_ID(random_integer), &rd);
}

void update_topic()
{
	/* generate a new random number for publication */
	struct random_integer_data rd = { .r = rand(), };

	/* publish the new data structure */
	orb_publish(ORB_ID(random_integer), topic_handle, &rd);
}

