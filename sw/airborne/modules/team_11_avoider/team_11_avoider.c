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
enum navigation_state_t navigation_state = SAFE;

float velocityx = 0.0, velocityy = 0.0;
float heading = 0;
float target_speed = 0.1;
uint8_t debug_enabled;
abi_event floor_detection_ev;
float ref_heading=0;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
		float fd_reference_heading) {
	ref_heading = fd_reference_heading;
	PRINT("FLOOR Received: %f\n", fd_reference_heading);
}

/*
 * Initialisation function
 */
void team_11_avoider_init(void) {

	srand(time(NULL));
	stepper_init(&velocityx, &velocityy);
	// bind our colorfilter callbacks to receive the color filter outputs
	AbiBindMsgFLOOR_DETECTION(FLOOR_DETECTION_ID, &floor_detection_ev,
			floor_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void team_11_avoider_periodic(void) {
	//PRINT("STATE: %d COUNT: %d, X: %d, Y: %d setX: %f, setY: %f \n", navigation_state, floor_count, floor_x, floor_y, xsetting, ysetting);
	// Only run the mudule if we are in the correct flight mode
	if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
		return;
	}
	if (debug_enabled) {
		stepper_periodic();
		guidance_h_set_guided_heading(heading);
	} else {
		switch (navigation_state) {
		case SAFE:
			velocityx = target_speed;
			guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi+ref_heading/50.0);
			break;
		case AVOID:
			break;
		default:
			break;
		}
	}
	guidance_h_set_guided_body_vel(velocityx, velocityy);
	return;
}

