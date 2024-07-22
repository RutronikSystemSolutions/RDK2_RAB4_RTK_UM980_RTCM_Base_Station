/*
 * hal_timer.h
 *
 *  Created on: 11 Sep 2023
 *      Author: jorda
 *
 * Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
 * including the software is for testing purposes only and,
 * because it has limited functions and limited resilience, is not suitable
 * for permanent use under real conditions. If the evaluation board is
 * nevertheless used under real conditions, this is done at one’s responsibility;
 * any liability of Rutronik is insofar excluded
 */

#ifndef HAL_HAL_TIMER_H_
#define HAL_HAL_TIMER_H_

#include <stdint.h>

int hal_timer_init();

uint32_t hal_timer_get_uticks(void);

#endif /* HAL_HAL_TIMER_H_ */
