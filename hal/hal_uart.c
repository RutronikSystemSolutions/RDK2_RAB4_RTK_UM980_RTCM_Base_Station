/*
 * hal_uart.c
 *
 *  Created on: 14 Nov 2023
 *      Author: jorda
 *
 * Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
 * including the software is for testing purposes only and,
 * because it has limited functions and limited resilience, is not suitable
 * for permanent use under real conditions. If the evaluation board is
 * nevertheless used under real conditions, this is done at one’s responsibility;
 * any liability of Rutronik is insofar excluded
 */


#include "hal_uart.h"

#include "cyhal_uart.h"
#include "cycfg_pins.h"
#include "hal_timer.h"

// UART context variables
cyhal_uart_t uart_obj;

static volatile uint8_t event_occured = 0;

// Initialize the ARDUINO UART configuration structure
const cyhal_uart_cfg_t uart_config =
{
    .data_bits = 8,
    .stop_bits = 1,
    .parity = CYHAL_UART_PARITY_NONE,
    .rx_buffer = NULL,
    .rx_buffer_size = 0
};

void hal_uart_callback(void *callback_arg, cyhal_uart_event_t event)
{
	event_occured = 1;
}

int hal_uart_init()
{
	cy_rslt_t result = cyhal_uart_init(&uart_obj, ARDU_TX, ARDU_RX, NC, NC, NULL, &uart_config);
	if (result != CY_RSLT_SUCCESS) return -1;

	uint32_t baudrate = 115200;
	result = cyhal_uart_set_baud(&uart_obj, baudrate, &baudrate);
	if (result != CY_RSLT_SUCCESS) return -2;

	cyhal_uart_register_callback(&uart_obj, hal_uart_callback, NULL);
	cyhal_uart_enable_event(&uart_obj, CYHAL_UART_IRQ_RX_DONE , 10, 1);

	return 0;
}

uint32_t hal_uart_readable(void)
{
	return cyhal_uart_readable(&uart_obj);
}

int hal_uart_read(uint8_t* buffer, uint16_t size)
{
	const uint32_t timeout_us = 100000; // 100ms
	size_t toread = (size_t)size;
	cy_rslt_t result = cyhal_uart_read_async(&uart_obj, buffer, toread);
	if (result != CY_RSLT_SUCCESS) return -1;

	uint32_t start_time = hal_timer_get_uticks();
	for(;;)
	{
		if (event_occured != 0)
		{
			event_occured = 0;
			break;
		}

		// Check for timeout
		uint32_t current_time = hal_timer_get_uticks();
		if (current_time < start_time) return -3;
		if ((current_time - start_time) > timeout_us)
		{
			return -2;
		}
	}
	return (int)toread;
}

int hal_uart_write(uint8_t* buffer, uint16_t size)
{
	size_t towrite = (size_t)size;
	cy_rslt_t result = cyhal_uart_write(&uart_obj, buffer, &towrite);
	if (result != CY_RSLT_SUCCESS) return -1;
	return (int)towrite;
}
