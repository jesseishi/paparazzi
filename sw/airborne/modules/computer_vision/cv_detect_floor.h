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

#endif /* SW_AIRBORNE_MODULES_COMPUTER_VISION_CV_DETECT_FLOOR_H_ */
