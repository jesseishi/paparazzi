/*
 * detec_floor.h
 *
 *  Created on: Mar 12, 2021
 *      Author: adam
 */

#ifndef SW_AIRBORNE_MODULES_COMPUTER_VISION_CV_DETECT_FLOOR_H_
#define SW_AIRBORNE_MODULES_COMPUTER_VISION_CV_DETECT_FLOOR_H_

#include <stdint.h>
#include <stdbool.h>

extern float fd_test_setting;
extern void floor_detector_init(void);
extern void floor_detector_periodic(void);

extern uint8_t orange_real_y_low;
extern uint8_t orange_real_y_high;
extern uint8_t orange_real_u_low ;
extern uint8_t orange_real_u_high;
extern uint8_t orange_real_v_low;
extern uint8_t orange_real_v_high;
extern uint8_t green_real_y_low;
extern uint8_t green_real_y_high;
extern uint8_t green_real_u_low;
extern uint8_t green_real_u_high;
extern uint8_t green_real_v_low;
extern uint8_t green_real_v_high;
extern uint8_t orange_sim_y_low;
extern uint8_t orange_sim_y_high;
extern uint8_t orange_sim_u_low;
extern uint8_t orange_sim_u_high;
extern uint8_t orange_sim_v_low;
extern uint8_t orange_sim_v_high;
extern uint8_t green_sim_y_low;
extern uint8_t green_sim_y_high;
extern uint8_t green_sim_u_low;
extern uint8_t green_sim_u_high;
extern uint8_t green_sim_v_low;
extern uint8_t green_sim_v_high;

extern float magic_scale_factor;

#endif /* SW_AIRBORNE_MODULES_COMPUTER_VISION_CV_DETECT_FLOOR_H_ */
