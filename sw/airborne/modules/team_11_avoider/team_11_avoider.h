/*
 * team_11_avoider.h
 *
 *  Created on: Mar 3, 2021
 *      Author: adam
 */

#ifndef SW_AIRBORNE_MODULES_TEAM_11_AVOIDER_TEAM_11_AVOIDER_H_
#define SW_AIRBORNE_MODULES_TEAM_11_AVOIDER_TEAM_11_AVOIDER_H_

/*
 * Something about the module
 */

#include "defines.h"
#include "stepper.h"
// settings


extern void team_11_avoider_init(void);
extern void team_11_avoider_periodic(void);

extern int floor_upper_treshold;
extern int floor_lower_treshold;
extern float gain;
extern uint8_t debug_enabled;


#endif /* SW_AIRBORNE_MODULES_TEAM_11_AVOIDER_TEAM_11_AVOIDER_H_ */
