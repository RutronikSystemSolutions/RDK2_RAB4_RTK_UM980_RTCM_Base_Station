/*
 * rtcm_packet.c
 *
 *  Created on: 23 Oct 2023
 *      Author: jorda
 *
 * Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
 * including the software is for testing purposes only and,
 * because it has limited functions and limited resilience, is not suitable
 * for permanent use under real conditions. If the evaluation board is
 * nevertheless used under real conditions, this is done at one’s responsibility;
 * any liability of Rutronik is insofar excluded
 */

#include "rtcm_packet.h"


uint16_t rtcm_packet_get_variable_size(uint8_t* buffer)
{
	return ((buffer[1] & 3) << 8) | buffer[2];
}

uint16_t rtcm_packet_get_type(uint8_t* buffer)
{
	return ((buffer[3]) << 4) | ((buffer[4] & 0xF0) >> 4);
}


