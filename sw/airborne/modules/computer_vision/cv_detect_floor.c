/*
 * detect_floor.c
 *
 *  Created on: Mar 12, 2021
 *      Author: adam
 */

#include "modules/computer_vision/cv_detect_floor.h"
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[floor_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;
float fd_test_setting = 0.0;
#ifndef FLOOR_OBJECT_DETECTOR_FPS
#define FLOOR_OBJECT_DETECTOR_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

static struct image_t *floor_detector(struct image_t *img){
	return img;
}

void floor_detector_init(void)
{
	/*Front camera always*/
	cv_add_to_device(&front_camera, floor_detector, FLOOR_OBJECT_DETECTOR_FPS);
}
void floor_detector_periodic(void)
{
	AbiSendMsgFLOOR_DETECTION(FLOOR_DETECTION_ID, fd_test_setting);
}
