/*
 * defines.h
 *
 *  Created on: Mar 6, 2021
 *      Author: adam
 */

#ifndef SW_AIRBORNE_MODULES_TEAM_11_AVOIDER_DEFINES_H_
#define SW_AIRBORNE_MODULES_TEAM_11_AVOIDER_DEFINES_H_

#include <stdio.h>
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"


#define ORANGE_AVOIDER_VERBOSE FALSE

//PRINT function made less verbose
#define PRINT(string,...) fprintf(stderr, "LOG: " string, ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif




#endif /* SW_AIRBORNE_MODULES_TEAM_11_AVOIDER_DEFINES_H_ */
