/*
 * stepper.h
 *
 *  Created on: Mar 6, 2021
 *      Author: adam
 */

#ifndef SW_AIRBORNE_MODULES_TEAM_11_AVOIDER_STEPPER_H_
#define SW_AIRBORNE_MODULES_TEAM_11_AVOIDER_STEPPER_H_

#include "defines.h"

void stepper_init(float*, float *);
void stepper_reset();
void stepper_periodic();
void stepper_enable();
void stepper_disable();
void stepper_set_vel(float, float);

extern int stepsx, stepsy;
extern float stepper_velx, stepper_vely;


#endif /* SW_AIRBORNE_MODULES_TEAM_11_AVOIDER_STEPPER_H_ */
