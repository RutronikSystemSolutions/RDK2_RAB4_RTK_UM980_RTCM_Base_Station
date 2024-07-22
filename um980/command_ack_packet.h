/*
 * command_ack_packet.h
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

#ifndef UM980_COMMAND_ACK_PACKET_H_
#define UM980_COMMAND_ACK_PACKET_H_

#include <stdint.h>

#define COMMAND_ACK_COMMAND_MAX_LENGTH	128
#define COMMAND_ACK_OK		0
#define COMMAND_ACK_NOT_OK	1

typedef struct
{
	uint8_t command[COMMAND_ACK_COMMAND_MAX_LENGTH];
	uint8_t status;
} command_ack_packet_t;

/**
 * @brief Given a valid NMEA packet, extract the command ack data
 *
 * @param [in] buffer Buffer containing the raw data string. Example: $command,unlog,response: OK*21
 * @param [in] len Length of the buffer
 * @param [out] command_ack_data Pointer to a structure that will be filled
 *
 * @retval 0 Success
 * @retval -1 Cannot extract information (unvalid packet?)
 */
int command_ack_packet_extract_data(uint8_t* buffer, uint16_t len, command_ack_packet_t* command_ack_data);

/**
 * @brief Check if the ack packet corresponds to the cmd given as parameter. Check also the status (OK or not)
 *
 * @param [in] cmd Command to be checked (example: unlog)
 * @param [in] data Data packet that has been parsed using command_ack_packet_extract_data
 *
 * @retval 0 Data packet corresponds to the command and status is OK
 */
int command_ack_packet_check_command_status(char* cmd, command_ack_packet_t* data);


#endif /* UM980_COMMAND_ACK_PACKET_H_ */
