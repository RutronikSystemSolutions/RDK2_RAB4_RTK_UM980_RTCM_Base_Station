/*
 * command_ack_packet.c
 *
 *  Created on: 20 Oct 2023
 *      Author: jorda
 *
 * Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
 * including the software is for testing purposes only and,
 * because it has limited functions and limited resilience, is not suitable
 * for permanent use under real conditions. If the evaluation board is
 * nevertheless used under real conditions, this is done at one’s responsibility;
 * any liability of Rutronik is insofar excluded
 */

#include "command_ack_packet.h"
#include "strutils.h"

#include <string.h>

#define OK_RESPONSE_LEN	12

int command_ack_packet_extract_data(uint8_t* buffer, uint16_t len, command_ack_packet_t* command_ack_data)
{
	// Packet:  $command,unlog,response: OK*21
	// Packet:  $command,umlog,response: PARSING FAILD NO MATCHING FUNC  UMLOG*60

	static const int ack_segment_count = 3;

	uint16_t seg_addr = 0;
	uint16_t seg_len = 0;
	uint16_t index = 0;

	// Check if enough segments
	int segment_count = get_segment_count(buffer, len, ',');
	if (segment_count != ack_segment_count)
	{
		return -1;
	}

	// Extract command
	get_segment_address_and_length(buffer, len, ',', 1, &seg_addr, &seg_len);
	for(uint16_t i = 0; (i < seg_len) && (i < COMMAND_ACK_COMMAND_MAX_LENGTH - 2); ++i)
	{
		command_ack_data->command[i] = buffer[seg_addr + i];
		index++;
	}
	command_ack_data->command[index] = '\0';

	// Extract status
	get_segment_address_and_length(buffer, len, ',', 2, &seg_addr, &seg_len);
	if (seg_len >= OK_RESPONSE_LEN)
	{
		char response[OK_RESPONSE_LEN + 1] = {0};
		for(uint16_t i = 0; i < OK_RESPONSE_LEN; ++i)
		{
			response[i] = buffer[seg_addr + i];
		}
		response[OK_RESPONSE_LEN] = '\0';

		if (strncmp("response: OK", response, OK_RESPONSE_LEN) == 0)
		{
			command_ack_data->status = COMMAND_ACK_OK;
		}
		else
		{
			command_ack_data->status = COMMAND_ACK_NOT_OK;
		}
	}
	else
	{
		command_ack_data->status = COMMAND_ACK_NOT_OK;
	}

	return 0;
}

int command_ack_packet_check_command_status(char* cmd, command_ack_packet_t* data)
{
	size_t cmd_len = strlen(cmd);

	if ((cmd_len <= 0) || (cmd_len >= COMMAND_ACK_COMMAND_MAX_LENGTH)) return -1;

	if (cmd_len != strlen((char*)data->command)) return -2;

	if (strncmp(cmd, (char*)data->command, cmd_len) != 0) return -3;

	if (data->status != COMMAND_ACK_OK) return -4;

	return 0;
}
