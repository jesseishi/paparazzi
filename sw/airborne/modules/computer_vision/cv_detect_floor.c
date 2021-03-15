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

#ifndef FLOOR_OBJECT_DETECTOR_FPS
#define FLOOR_OBJECT_DETECTOR_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

//h = 520, w = 240
#define image_h 520
#define image_w 240

typedef enum {
	orange_real, green_real, orange_sim, green_sim
} color_t;

uint8_t orange_real_y_low = 40;
uint8_t orange_real_y_high = 200;
uint8_t orange_real_u_low = 0;
uint8_t orange_real_u_high = 150;
uint8_t orange_real_v_low = 140;
uint8_t orange_real_v_high = 220;
uint8_t green_real_y_low = 70;
uint8_t green_real_y_high = 95;
uint8_t green_real_u_low = 100;
uint8_t green_real_u_high = 150;
uint8_t green_real_v_low = 100;
uint8_t green_real_v_high = 132;
uint8_t orange_sim_y_low = 40;
uint8_t orange_sim_y_high = 200;
uint8_t orange_sim_u_low = 0;
uint8_t orange_sim_u_high = 150;
uint8_t orange_sim_v_low = 140;
uint8_t orange_sim_v_high = 220;
uint8_t green_sim_y_low = 0;
uint8_t green_sim_y_high = 100;
uint8_t green_sim_u_low = 0;
uint8_t green_sim_u_high = 120;
uint8_t green_sim_v_low = 0;
uint8_t green_sim_v_high = 130;

static pthread_mutex_t mutex;
float fd_reference_heading = 0.0;

//typedef bool filtered_image_t[image_h][image_w];
bool filtered_image[4];

// Declare functions.
void count_per_heading(bool filtered[image_h][image_w], uint16_t *sum,
                       uint16_t h, uint16_t w);

void mat_eliminator(bool filtered[image_h][image_w]);

void color_filter(struct image_t *img, color_t color,
                  bool filtered[image_h][image_w]);

static struct image_t *floor_detector(struct image_t *img);

// Define functions.
void floor_detector_init(void) {
	/*Front camera always*/
	cv_add_to_device(&front_camera, floor_detector, FLOOR_OBJECT_DETECTOR_FPS);
}

void floor_detector_periodic(void) {
	AbiSendMsgFLOOR_DETECTION(FLOOR_DETECTION_ID, fd_reference_heading);
}

static struct image_t *floor_detector(struct image_t *img) {

	// img is the image the front camera sees, but rotated by 90 deg counter clockwise.
	// However, the width and height (and corresponding [x,y] coordinates) are defined
	// on this rotated image as normally. Furthermore, the image buffer (as used in color_filter())
	// is defined as a 1D array with slices in width. So to stay consistent with that,
	// we'll make 2D arrays with [image_h][image_w].
	bool im_floor[image_h][image_w];
	color_filter(img, green_sim, im_floor);

	mat_eliminator(im_floor);

	uint16_t floor_per_heading[image_h]; // Since the image is rotated, the height spans all headings.
	count_per_heading(im_floor, floor_per_heading, image_h, image_w);
	//Copy the image, but be careful, only the pointer to the buffer
	//has been copied
	//struct image_t slice_image = *img; //something wrong with that
	//TODO. Create new buffer to make a slice
	//and call color filter once again
	//img->w = 1;

	uint16_t orange_per_heading[image_h];	//not initialized now
	//memset(orange_per_heading,0,image_h*sizeof(orange_per_heading[0]));
	int16_t cost[image_h];
	for (int i = 0; i < image_h; i++) {
		cost[i] = -floor_per_heading[i] * (1 - orange_per_heading[i]);
	}

	int rolling_length = 25;
	float cost_rolling_ave[image_h];
	cost_rolling_ave[0] = (float) cost[0] / rolling_length;
	for (int i = 1; i < image_h; i++) {
		cost_rolling_ave[i] = cost_rolling_ave[i - 1]
				+ (float) cost[i] / rolling_length; // TODO: - cost[i-25] / rolling_length or something like that.
	}
	int16_t best_heading = 0;
	//calculate minimum
	float magic_scale_factor = 0.1;
	fd_reference_heading = (float)(best_heading - image_h / 2)* magic_scale_factor;
	return img; //TODO: draw in the image so that we can see what this algorithm did.

}

void color_filter(struct image_t *img, color_t color,
                  bool filtered[image_h][image_w]) {

	uint8_t y_low = 0;
	uint8_t y_high = 0;
	uint8_t u_low = 0;
	uint8_t u_high = 0;
	uint8_t v_low = 0;
	uint8_t v_high = 0;

	switch (color) {
	case orange_real:
		y_low = orange_real_y_low;
		y_high = orange_real_y_high;
		u_low = orange_real_u_low;
		u_high = orange_real_u_high;
		v_low = orange_real_v_low;
		v_high = orange_real_v_high;
		break;
	case green_real:
		y_low = green_real_y_low;
		y_high = green_real_y_high;
		u_low = green_real_u_low;
		u_high = green_real_u_high;
		v_low = green_real_v_low;
		v_high = green_real_v_high;
		break;
	case orange_sim:
		y_low = orange_sim_y_low;
		y_high = orange_sim_y_high;
		u_low = orange_sim_u_low;
		u_high = orange_sim_u_high;
		v_low = orange_sim_v_low;
		v_high = orange_sim_v_high;
		break;
	case green_sim:
		y_low = green_sim_y_low;
		y_high = green_sim_y_high;
		u_low = green_sim_u_low;
		u_high = green_sim_u_high;
		v_low = green_sim_v_low;
		v_high = green_sim_v_high;
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
			if ((*yp >= y_low) && (*yp <= y_high) && (*up >= u_low)
					&& (*up <= u_high) && (*vp >= v_low) && (*vp <= v_high)) {
				filtered[y][x] = true;
			} else {
				filtered[y][x] = false;
			}
		}
	}

	return;
}

void mat_eliminator(bool filtered[image_h][image_w]) {
	/*Mat eliminator*/
	for (int y = 0; y < image_h; y++) {
		for (int x = 1; x < image_w / 2; x++) {
			if (filtered[y][x] == true && filtered[y][x - 1] == false) {
				for (int i = 0; i <= x; i++) {
					filtered[y][i] = true;
				}
			}
		}
	}
	return;
}
void count_per_heading(bool filtered[image_h][image_w], uint16_t *sum,
		uint16_t h, uint16_t w) {

	for (int y = 0; y < h; y++) {
		sum[y] = 0;
		for (int x = 0; x < w; x++) {
			sum[y] += filtered[y][x];
		}
	}
	return;
}
