/*
 * stepper.c
 *
 *  Created on: Mar 6, 2021
 *      Author: adam
 */
#include "stepper.h"

int stepsx;
int stepsy;
static uint8_t stepper_enabled = 0;
float stepper_velx = 0.1, stepper_vely = 0.1;
static float *xsetting;
static float *ysetting;

void stepper_init(float *xset, float *yset) {

	xsetting = xset;
	ysetting = yset;
}

void stepper_reset() {
	stepsx = 0;
	stepsy = 0;
}
void stepper_periodic() {

	if (stepsx > 0) {
		*xsetting = stepper_velx;
		stepsx--;
	} else if (stepsx < 0) {
		*xsetting = -stepper_velx;
		stepsx++;
	} else {
		*xsetting = 0;
	}
	if (stepsy > 0) {
		*ysetting = stepper_vely;
		stepsy--;
	} else if (stepsy < 0) {
		*ysetting = -stepper_vely;
		stepsy++;
	} else {
		*ysetting = 0;
	}

}
void stepper_enable() {
	stepper_enabled = 1;
}
void stepper_disable() {
	stepper_enabled = 0;
}
void stepper_set_vel(float x, float y) {
	if (x > 0.0 && y > 0.0) {
		stepper_velx = x;
		stepper_vely = y;
	} else {
		PRINT("ERROR: Stepper velocities must be positive");
	}
}

