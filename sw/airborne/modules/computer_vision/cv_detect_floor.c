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

typedef enum {
	orange_real, green_real, orange_sim, green_sim
} color_t;

#define image_h 520
#define image_w 240

typedef bool filtered_image_t[image_h][image_w];
bool filtered_image[4];

void count_per_heading(filtered_image_t* filtered, uint16_t *sum);

void mat_eliminator(filtered_image_t* filtered);

void color_filter(struct image_t *img, color_t color,
		filtered_image_t *filtered);
static struct image_t *floor_detector(struct image_t *img);

void floor_detector_init(void) {
	/*Front camera always*/
	cv_add_to_device(&front_camera, floor_detector, FLOOR_OBJECT_DETECTOR_FPS);
}
void floor_detector_periodic(void) {
	AbiSendMsgFLOOR_DETECTION(FLOOR_DETECTION_ID, fd_test_setting);
}
static struct image_t *floor_detector(struct image_t *img) {

	filtered_image_t im_floor;
	color_filter(*img, green_real, &im_floor);
	mat_eliminator(&im_floor);
	uint16_t floor_per_heading[image_h]; //or image_v
	count_per_heading(im_floor, &floor_per_heading);
	//Copy the image, but be careful, only the pointer to the buffer
	//has been copied
	struct image_t slice_image = img;
	/*TODO. Create new buffer and change
	 * sizes
	 */

	//int cost = -floor_per_heading * (1-orange_per_heading)
	return img;
}

//h = 520, w = 240
void color_filter(struct image_t *img, color_t color,
		filtered_image_t *filtered) {

	uint8_t y_low = 0;
	uint8_t y_high = 0;
	uint8_t u_low = 0;
	uint8_t u_high = 0;
	uint8_t v_low = 0;
	uint8_t v_high = 0;

	switch (color) {
	case orange_real:
		y_low = 40;
		y_high = 200;
		u_low = 0;
		u_high = 150;
		v_low = 140;
		v_high = 220;
		break;
	case green_real:
		y_low = 70;
		y_high = 95;
		u_low = 100;
		u_high = 150;
		v_low = 100;
		v_high = 132;
		break;
	case orange_sim:
		y_low = 40;
		y_high = 200;
		u_low = 0;
		u_high = 150;
		v_low = 140;
		v_high = 220;
		break;
	case green_sim:
		y_low = 0;
		y_high = 100;
		u_low = 0;
		u_high = 120;
		v_low = 0;
		v_high = 130;
		break;
	default:
		PRINT("Don't have the color build in yet");
		break;
	}
	uint8_t *buffer = img->buf;
	// Go through all the pixels
	for (uint16_t y = 0; y < img->h; y++) {
		for (uint16_t x = 0; x < img->w; x++) {
			// Check if the color is inside the specified values
			uint8_t *yp, *up, *vp;
			if (x % 2 == 0) {
				// Even x
				up = &buffer[y * 2 * img->w + 2 * x];      // U
				yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
				vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
				//yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
			} else {
				// Uneven x
				up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
				//yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
				vp = &buffer[y * 2 * img->w + 2 * x];      // V
				yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
			}
			if ((*yp >= y_low) && (*yp <= y_high) && (*up >= v_low)
					&& (*up <= u_high) && (*vp >= v_low) && (*vp <= v_high)) {
				*filtered[x][y] = true;
			} else {
				*filtered[x][y] = false;
			}
			/*				if (draw) {
			 *yp = 255;  // make pixel brighter in image
			 }*/
		}
	}
}

void mat_eliminator(filtered_image_t* filtered) {
	/*Mat eliminator*/
	for (int y = 0; y < image_h; y++) {
		for (int x = 0; x < image_w / 2; x++) {
			if (filtered[x][y] == true && filtered[x - 1][y] == false) {
				for (int i = 0; i < x; i++) {
					filtered[i][y] = true;
				}
			}
		}
	}
}
void count_per_heading(filtered_image_t* filtered, uint16_t *sum) {

}
