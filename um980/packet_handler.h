/*
 * packet_handler.h
 *
 *  Created on: 21 Oct 2023
 *      Author: jorda
 *
 * Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
 * including the software is for testing purposes only and,
 * because it has limited functions and limited resilience, is not suitable
 * for permanent use under real conditions. If the evaluation board is
 * nevertheless used under real conditions, this is done at one’s responsibility;
 * any liability of Rutronik is insofar excluded
 */

#ifndef UM980_PACKET_HANDLER_H_
#define UM980_PACKET_HANDLER_H_

#include <stdint.h>

#define PACKET_HANDLER_ERROR_READ				-1
#define PACKET_HANDLER_ERROR_BUFFER_FULL		-2
#define PACKET_HANDLER_ERROR_NO_PACKET			-3
#define PACKET_HANDLER_ERROR_BUFFER_TOO_SMALL	-4
#define PACKET_HANDLER_ERROR_INVALID_NMEA_LEN	-5

#define PACKET_HANDLER_NMEA_PACKET		1
#define PACKET_HANDLER_RTCM_PACKET		2
#define PACKET_HANDLER_UNKNOWN_PACKET	3

typedef uint32_t (*um980_app_uart_readable_func_t)(void);
typedef int (*um980_app_uart_read_func_t)(uint8_t* buffer, uint16_t size);
typedef int (*um980_app_uart_write_func_t)(uint8_t* buffer, uint16_t size);
typedef uint32_t (*um980_app_get_uticks)(void);

/**
 * @brief Initialize the module
 */
void packet_handler_init(um980_app_uart_readable_func_t uart_readable,
		um980_app_uart_read_func_t uart_read,
		um980_app_uart_write_func_t uart_write);

/**
 * @brief Read a packet out of the data stream
 *
 * The function first read from the UART connection and fill an internal buffer
 * Then it reads the first packet available in the datastream
 * It can be a RTCM packet (starts with 0xD3 or a NMEA packet that starts with '$')
 *
 * @retval 0 Nothing to read
 * @retval > 0 A packet has been read. Retval contains the length of the packet
 * @retval < 0  Error occured
 */
int packet_handler_read_packet(uint8_t* buffer, uint16_t buffer_len);

/**
 * @brief Reset the internal buffer
 */
void packet_handler_reset();

/**
 * @brief Get the type of the packet (NMEA, RTCM or Unknown)
 */
uint16_t packet_handler_get_packet_type(uint8_t* buffer);

#endif /* UM980_PACKET_HANDLER_H_ */
