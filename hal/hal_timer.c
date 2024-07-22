/*
 * hal_timer.c
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

#include "hal_timer.h"
#include "cyhal_timer.h"

static cyhal_timer_t Systick_obj;

int hal_timer_init()
{
	const cyhal_timer_cfg_t Systick_cfg =
	{
		.compare_value = 0,                  // Timer compare value, not used
		.period        = 0xFFFFFFFFUL,              	// Timer period set to a large enough value
		//   compared to event being measured
		.direction     = CYHAL_TIMER_DIR_UP, // Timer counts up
		.is_compare    = false,              // Don't use compare mode
		.is_continuous = false,              // Do not run timer indefinitely
		.value         = 0                   // Initial value of counter
	};


	// Initialize the timer object. Does not use pin output ('pin' is NC) and does not use a
	// pre-configured clock source ('clk' is NULL).
	cyhal_timer_init(&Systick_obj, NC, NULL);
	cyhal_timer_configure(&Systick_obj, &Systick_cfg);
	cyhal_timer_set_frequency(&Systick_obj, 1000000);
	cyhal_timer_start(&Systick_obj);

	return 0;
}

uint32_t hal_timer_get_uticks(void)
{
	return cyhal_timer_read(&Systick_obj);
}


