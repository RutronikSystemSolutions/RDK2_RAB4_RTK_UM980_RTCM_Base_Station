/*
 * packet_handler.c
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

#include "packet_handler.h"

#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "rtcm_packet.h"

#define PACKET_HANDLER_BUFFER_SIZE 1024

static um980_app_uart_readable_func_t uart_readable_func = NULL;
static um980_app_uart_read_func_t uart_read_func = NULL;
static um980_app_uart_write_func_t uart_write_func = NULL;

// Buffer storing stream coming from the UM980 sensor (+1 because of internal handling -> we add \0 for debug purposes after reading something from the UART)
static uint8_t rx_buffer[PACKET_HANDLER_BUFFER_SIZE + 1];

// Store how much data is available inside the rx buffer
static uint16_t rx_buffer_len = 0;

static const uint16_t rtcm_fixed_size = 6;

void packet_handler_init(um980_app_uart_readable_func_t uart_readable,
		um980_app_uart_read_func_t uart_read,
		um980_app_uart_write_func_t uart_write)
{
	uart_readable_func = uart_readable;
	uart_read_func = uart_read;
	uart_write_func = uart_write;
}

/**
 * @brief Read everything what is available on the UART inside the buffer
 *
 * @retval 0 Success
 * @retval -1 Buffer is full
 * @retval -2 error while reading
 */
static int read_available()
{
	uint32_t readable = uart_readable_func();
	if (readable == 0) return 0;

	uint16_t remaining_space = PACKET_HANDLER_BUFFER_SIZE - rx_buffer_len;
	uint16_t toread = (readable > remaining_space) ? remaining_space : (uint16_t) readable;

	if (toread == 0)
	{
		return PACKET_HANDLER_ERROR_BUFFER_FULL;
	}

	int retval = uart_read_func(&rx_buffer[rx_buffer_len], toread);
	if (retval < 0)
	{
		return PACKET_HANDLER_ERROR_READ;
	}

	rx_buffer_len += (uint16_t) retval;
	// Add \0 for debug option, in case the user use printf on the rx_buffer
	rx_buffer[rx_buffer_len] = '\0';

	return 0;
}

static bool is_nmea_packet_valid(uint8_t* buffer, uint16_t len)
{
	if (buffer[0] != '$') return false;

	for(uint16_t i = 1; i < len; ++i)
	{
		if ((buffer[i-1] == '\r') && (buffer[i] == '\n')) return true;
	}

	return false;
}

static uint16_t get_nmea_packet_len(uint8_t* buffer, uint16_t len)
{
	for(uint16_t i = 1; i < len; ++i)
	{
		if ((buffer[i-1] == '\r') && (buffer[i] == '\n')) return (i + 1);
	}
	return 0;
}

static uint32_t crc24(uint8_t* buffer, uint16_t size)
{
	uint32_t crc = 0;
	int i = 0;
	while (size--)
	{
		crc ^= (*buffer++) << (16);
		for (i = 0; i < 8; i++)
		{
			crc <<= 1;
			if (crc & 0x1000000)
				crc ^= 0x01864cfb;
		}
	}
	return crc;
}

/**
 * @brief Check if the packet starting at @start_addr in rx_buffer is valid RTCM or not
 *
 * @retval true Valid packet
 * @retval false Not valid packet
 */
static bool is_rtcm_packet_valid(uint8_t* buffer, uint16_t len)
{
	// Enough place for preamble, reserved bits, message length and CRC? (at least 6  bytes -> 48bits)
	if (len < rtcm_fixed_size) return false;

	if (buffer[0] != 0xD3) return false; // Correct header?

	if ((buffer[1] & 0xFC) != 0) return false; // reserved 6 bits should be 0

	// Extract packet length (10 bits)
	uint16_t packet_len = rtcm_packet_get_variable_size(buffer);

	// Enough?
	if (len < (rtcm_fixed_size + packet_len)) return false;

	// Compute the CRC
	uint32_t crc = crc24(buffer, packet_len + 3);

	uint32_t crc_is = (buffer[3 + packet_len] << 16)
		| (buffer[3 + packet_len + 1] << 8)
		| (buffer[3 + packet_len + 2]);

	if (crc != crc_is) return false;

	return true;
}

void packet_handler_reset()
{
	rx_buffer_len = 0;
}

int packet_handler_read_packet(uint8_t* buffer, uint16_t buffer_len)
{
	int retval = read_available();
	if (retval != 0) return retval;

	// Search for $ or D3
	for(int i = 0; i < rx_buffer_len; ++i)
	{
		if (rx_buffer[i] == '$')
		{
			if (is_nmea_packet_valid(&rx_buffer[i], (rx_buffer_len -i)) == true)
			{
				uint16_t packet_len = get_nmea_packet_len(&rx_buffer[i], (rx_buffer_len -i));
				if (packet_len == 0) return PACKET_HANDLER_ERROR_INVALID_NMEA_LEN;

				if (buffer_len < (packet_len + 1))
				{
					return PACKET_HANDLER_ERROR_BUFFER_TOO_SMALL;
				}

				// Read to buffer
				memcpy(buffer, &rx_buffer[i], packet_len);
				buffer[packet_len] = '\0';

				// Shift RX buffer to keep it clean and update len
				memcpy(rx_buffer, &rx_buffer[i + packet_len], rx_buffer_len - (i + packet_len));
				rx_buffer_len -= (packet_len + i);

				return packet_len;
			}

			// Packet not valid, wait
			return 0;
		}
		else if (rx_buffer[i] == 0xD3)
		{
			// Starts with 0xD3 -> RTCM
			if (is_rtcm_packet_valid(&rx_buffer[i], (rx_buffer_len - i)) == true)
			{
				uint16_t packet_len = rtcm_packet_get_variable_size(&rx_buffer[i]) + rtcm_fixed_size;
				if (buffer_len < (packet_len + 1))
				{
					return PACKET_HANDLER_ERROR_BUFFER_TOO_SMALL;
				}

				// Read to buffer
				memcpy(buffer, &rx_buffer[i], packet_len);
				buffer[packet_len] = '\0';

				// Shift RX buffer to keep it clean and update len
				memcpy(rx_buffer, &rx_buffer[i + packet_len], rx_buffer_len - (i + packet_len));
				rx_buffer_len -= (packet_len + i);

				return packet_len;
			}

			// Packet not valid, wait
			return 0;
		}
	}

	return 0;
}

uint16_t packet_handler_get_packet_type(uint8_t* buffer)
{
	if (buffer[0] == '$') return PACKET_HANDLER_NMEA_PACKET;
	if (buffer[0] == 0xD3) return PACKET_HANDLER_RTCM_PACKET;
	return PACKET_HANDLER_UNKNOWN_PACKET;
}
