/*
 * usb_transfer.c
 *
 *  Created on: 3 Oct 2023
 *      Author: jorda
 *
 * Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
 * including the software is for testing purposes only and,
 * because it has limited functions and limited resilience, is not suitable
 * for permanent use under real conditions. If the evaluation board is
 * nevertheless used under real conditions, this is done at one’s responsibility;
 * any liability of Rutronik is insofar excluded
 */

#include "usb_transfer.h"

#include <stdlib.h>
#include <string.h>

#undef DEBUG_USB_TRANSFER

#ifdef DEBUG_USB_TRANSFER
#include <stdio.h>
#endif

static uint8_t* send_buffer = NULL;
static uint16_t to_send = 0;
static uint16_t write_address = 0;
static uint16_t read_address = 0;
static uint16_t total_size = 0;
static uint8_t send_zero_packet = 0;

#ifdef DEBUG_USB_TRANSFER
void usb_transfer_dump()
{
	printf("size: %u \t to send: %u \t write address: %u \t read address: %u \r\n",
			total_size,
			to_send,
			write_address,
			read_address);
}
#endif

static usb_transfer_is_ready_func_t internal_is_ready = NULL;
static usb_transfer_put_data_func_t internal_put_data = NULL;

int usb_transfer_init(uint16_t buffer_length,
		usb_transfer_is_ready_func_t is_ready,
		usb_transfer_put_data_func_t put_data)
{
	send_buffer = malloc(buffer_length);
	if (send_buffer == NULL) return -1;
	total_size = buffer_length;
	internal_is_ready = is_ready;
	internal_put_data = put_data;
	return 0;
}

/**
 * @brief Copy a buffer inside the internal ring buffer
 *
 * First fill the internal ring buffer. Once the ring buffer is full, start back at address 0
 */
static void write_to_buffer(uint8_t* buffer, uint16_t length)
{
	uint16_t first_chunk = total_size - write_address;
	uint16_t tocopy = (length < first_chunk) ? length : first_chunk;
	// Copy until the end of the buffer (or less)
	memcpy(&send_buffer[write_address], buffer, tocopy);
	if (tocopy < length)
	{
		// If something remaining, start back to the start of the buffer (ring buffer)
		memcpy(send_buffer, &buffer[tocopy], length - tocopy);
	}
}

/**
 * @brief Read data from the buffer
 */
static void read_from_buffer(uint8_t* buffer, uint16_t length)
{
	uint16_t first_chunk = total_size - read_address;
	uint16_t tocopy = (length < first_chunk) ? length : first_chunk;
	// Copy until the end of the buffer (or less)
	memcpy(buffer, &send_buffer[read_address], tocopy);
	if (tocopy < length)
	{
		// If something remaining, start back to the start of the buffer (ring buffer)
		memcpy(&buffer[tocopy], send_buffer, length - tocopy);
	}
}

int usb_transfer_send(uint8_t* buffer, uint16_t length)
{
	if (send_buffer == NULL) return -1;

	uint16_t remaining_place = total_size - to_send;
	if (length > remaining_place) return -2;

	write_to_buffer(buffer, length);

	to_send += length;
	write_address = (write_address + length) % total_size;

#ifdef DEBUG_USB_TRANSFER
	usb_transfer_dump();
#endif

	return 0;
}

void usb_transfer_do()
{
	uint8_t packet[64];

	if ((to_send == 0) && (send_zero_packet == 0))
	{
		// Nothing to do
		return;
	}

	// USB ready?
	if (internal_is_ready() == 0)
	{
		// No, not ready
		return;
	}

	if (send_zero_packet != 0)
	{
		internal_put_data(NULL, 0);
		send_zero_packet = 0;
		return;
	}

	uint8_t size = (to_send > 64) ? 64 : to_send;
	read_from_buffer(packet, size);

	to_send -= size;
	read_address = (read_address + size) % total_size;

#ifdef DEBUG_USB_TRANSFER
	usb_transfer_dump();
#endif

	internal_put_data(packet, size);
}

int usb_transfer_is_task_remaining()
{
	if (send_zero_packet != 0) return 1;
	if (read_address != write_address) return 1;
	return 0;
}
