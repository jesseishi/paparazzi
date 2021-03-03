/*
 * team_11_avoider.c
 *
 *  Created on: Mar 3, 2021
 *      Author: adam
 */
/*
 * Something about the module
 */

#include "modules/team_11_avoider/team_11_avoider.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

//PRINT function made less verbose
#define PRINT(string,...) fprintf(stderr, "LOG: " string, ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};


// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif


#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif


/*
 * Initialisation function
 */
void team_11_avoider_init(void)
{

  // bind our colorfilter callbacks to receive the color filter outputs
  //AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  //AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void team_11_avoider_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    //navigation_state = SEARCH_FOR_SAFE_HEADING;
    //obstacle_free_confidence = 0;
    return;
  }
  guidance_h_set_guided_heading_rate(1.0);
 return;
}

