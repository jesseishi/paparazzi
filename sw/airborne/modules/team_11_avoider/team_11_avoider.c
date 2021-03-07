/*
 * team_11_avoider.c
 *
 *  Created on: Mar 3, 2021
 *      Author: adam
 */
/*
 * Something about the module
 */

#include "team_11_avoider.h"

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include <stdio.h>
#include <time.h>

enum navigation_state_t {
	SAFE, AVOID
};
enum navigation_state_t navigation_state = AVOID;
abi_event bottom_cam_color_detection_ev;
int32_t floor_count;
int16_t floor_x;
int16_t floor_y;
float oag_color_count_frac = 0.1;
int floor_upper_treshold = 56600;
int floor_lower_treshold = 56100;
float gain = 0.2;
float velocityx = 0.0, velocityy = 0.0;
uint8_t debug_enabled;
float heading = 0;
float target_speed = 1;
static void bottom_cam_color_detection_cb(
		uint8_t __attribute__((unused)) sender_id, int16_t pixel_x,
		int16_t pixel_y, int16_t __attribute__((unused)) pixel_width,
		int16_t __attribute__((unused)) pixel_height, int32_t quality,
		int16_t __attribute__((unused)) extra) {
	floor_count = quality;
	floor_x = pixel_x;
	floor_y = pixel_y;

}

// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error Define ORANGE_AVOIDER_VISUAL_DETECTION_ID
#endif

#ifndef FLOOR_FRONT_CAM_VISUAL_DETECTION_ID
#error Define FLOOR_FRONT_CAM_VISUAL_DETECTION_ID
#endif

#ifndef FLOOR_BOTTOM_CAM_VISUAL_DETECTION_ID
#error Define FLOOR_BOTTOM_CAM_VISUAL_DETECTION_ID
#endif

/*
 * Initialisation function
 */
void team_11_avoider_init(void) {

	srand(time(NULL));
	stepper_init(&velocityx, &velocityy);
	// bind our colorfilter callbacks to receive the color filter outputs
	AbiBindMsgVISUAL_DETECTION(FLOOR_BOTTOM_CAM_VISUAL_DETECTION_ID,
			&bottom_cam_color_detection_ev, bottom_cam_color_detection_cb);
	//AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void team_11_avoider_periodic(void) {
	//PRINT("STATE: %d COUNT: %d, X: %d, Y: %d setX: %f, setY: %f \n", navigation_state, floor_count, floor_x, floor_y, xsetting, ysetting);
	// Only run the mudule if we are in the correct flight mode
	if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
		//navigation_state = SEARCH_FOR_SAFE_HEADING;
		//obstacle_free_confidence = 0;
		return;
	}
	if (debug_enabled) {
		stepper_periodic();
		guidance_h_set_guided_heading(heading);
	} else {
		switch (navigation_state) {
		case SAFE:
			PRINT("STATE: SAFE");
			if (floor_count < floor_lower_treshold) {
				navigation_state = AVOID;
			} else {
				//normalize speed
				float totalsetspeed = sqrtf(
						velocityx * velocityx + velocityy * velocityy);
				if (totalsetspeed > 0.1) {
					velocityx = velocityx * target_speed / totalsetspeed;
					velocityy = velocityy * target_speed / totalsetspeed;
				}
			}
			break;
		case AVOID:
			PRINT("STATE: AVOID");
			if (floor_count > floor_upper_treshold) {
				navigation_state = SAFE;

			} else {
				//x y axes are switched
				velocityx = gain * ((float) floor_y);
				velocityy = gain * ((float) floor_x);
			}
			break;
		default:
			break;
			}
		if (fabsf(velocityx) < 0.05 && fabsf(velocityy) < 0.05) {
						PRINT("WAKE-UP CALL\n");
						int direction = rand() % 365;
						float angle = ((float) direction) * M_PI / 180;

						velocityx = target_speed * cosf(angle);
						velocityy = target_speed * sinf(angle);
		}
	}

	guidance_h_set_guided_body_vel(velocityx, velocityy);
	PRINT("FILTER: %d %4f %4f\n", floor_count, velocityx, velocityy);
	return;
}

