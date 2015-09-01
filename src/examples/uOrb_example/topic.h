/*
 * topic.h
 *
 *  Created on: 19.08.2015
 *      Author: tetresch
 */

#ifndef TOPIC_H_
#define TOPIC_H_

#include <stdint.h>
#include <uORB/uORB.h>


/* define the data structure that will be published where subscribers can see it */
struct random_integer_data {
	int r;
};

/* register this as object request broker structure */
ORB_DECLARE(random_integer);

#endif /* TOPIC_H_ */
