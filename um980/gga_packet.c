/*
 * gga_packet.c
 *
 *  Created on: 19 Oct 2023
 *      Author: jorda
 *
 * Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
 * including the software is for testing purposes only and,
 * because it has limited functions and limited resilience, is not suitable
 * for permanent use under real conditions. If the evaluation board is
 * nevertheless used under real conditions, this is done at one’s responsibility;
 * any liability of Rutronik is insofar excluded
 */

#include "gga_packet.h"
#include "strutils.h"

#include <stdlib.h>

/**
 * From a valid GGA packet (defined by buffer / len) extract the relevant informations
 *
 * @retval 0 Success
 *
 */
int gga_packet_extract_data(uint8_t* buffer, uint16_t len, um980_gga_packet_t* gga_data)
{
	// $GNGGA,122917.00,4845.77916055,N,00758.32526162,E,7,17,0.8,130.1941,M,48.3746,M,,*7D
	static const int gga_segment_count = 15;

	uint16_t seg_addr = 0;
	uint16_t seg_len = 0;

	char strbuff[32] = {0};

	// Check if enough segments
	int segment_count = get_segment_count(buffer, len, ',');
	if (segment_count != gga_segment_count)
	{
		return -1;
	}

	// UTC time
	get_segment_address_and_length(buffer, len, ',', 1, &seg_addr, &seg_len);
	strbuff[0] = buffer[seg_addr];
	strbuff[1] = buffer[seg_addr + 1];
	strbuff[2] = '\0';
	gga_data->hours = (uint8_t) atoi(strbuff);

	strbuff[0] = buffer[seg_addr + 2];
	strbuff[1] = buffer[seg_addr + 3];
	strbuff[2] = '\0';
	gga_data->minutes = (uint8_t) atoi(strbuff);

	strbuff[0] = buffer[seg_addr + 4];
	strbuff[1] = buffer[seg_addr + 5];
	strbuff[2] = '\0';
	gga_data->seconds = (uint8_t) atoi(strbuff);

	// Latitude 3137.36664 becomes 31 degrees and 37.26664 seconds = 31 + 37.36664/60 = 31.6227773
	get_segment_address_and_length(buffer, len, ',', 2, &seg_addr, &seg_len);
	strbuff[0] = buffer[seg_addr];
	strbuff[1] = buffer[seg_addr + 1];
	strbuff[2] = '\0';
	gga_data->lat_degree = (double) atoi(strbuff);

	for(uint16_t i = 2; i < seg_len; ++i)
	{
		strbuff[i-2] = buffer[seg_addr + i];
	}
	strbuff[seg_len - 2] = '\0';
	gga_data->lat_seconds = atof(strbuff);

	// Latitude direction (N or S)
	get_segment_address_and_length(buffer, len, ',', 3, &seg_addr, &seg_len);
	if (buffer[seg_addr] == 'N') gga_data->lat_dir = 0;
	else gga_data->lat_dir = 1;

	// Longitude 00212.21149 becomes 2 degrees and 12.21149 seconds = 2 + 12.21149/60 = 2.20352483
	get_segment_address_and_length(buffer, len, ',', 4, &seg_addr, &seg_len);
	strbuff[0] = buffer[seg_addr];
	strbuff[1] = buffer[seg_addr + 1];
	strbuff[2] = buffer[seg_addr + 2];
	strbuff[3] = '\0';
	gga_data->lon_degree = (double) atoi(strbuff);

	for(uint16_t i = 3; i < seg_len; ++i)
	{
		strbuff[i-3] = buffer[seg_addr + i];
	}
	strbuff[seg_len - 3] = '\0';
	gga_data->lon_seconds = atof(strbuff);

	// Longitude direction (E or W)
	get_segment_address_and_length(buffer, len, ',', 5, &seg_addr, &seg_len);
	if (buffer[seg_addr] == 'E') gga_data->lon_dir = 0;
	else gga_data->lon_dir = 1;

	// Quality of the signal
	get_segment_address_and_length(buffer, len, ',', 6, &seg_addr, &seg_len);
	strbuff[0] = buffer[seg_addr];
	strbuff[1] = '\0';
	gga_data->quality = (uint8_t) atoi(strbuff);

	// Satellites in use
	get_segment_address_and_length(buffer, len, ',', 7, &seg_addr, &seg_len);
	for(uint16_t i = 0; i < seg_len; ++i)
	{
		strbuff[i] = buffer[seg_addr + i];
	}
	strbuff[seg_len] = '\0';
	gga_data->satellites_in_use = (uint8_t) atoi(strbuff);

	// Horizontal dilution of precision
	get_segment_address_and_length(buffer, len, ',', 8, &seg_addr, &seg_len);
	for(uint16_t i = 0; i < seg_len; ++i)
	{
		strbuff[i] = buffer[seg_addr + i];
	}
	strbuff[seg_len] = '\0';
	gga_data->hdop = atof(strbuff);

	// Altitude
	get_segment_address_and_length(buffer, len, ',', 9, &seg_addr, &seg_len);
	for(uint16_t i = 0; i < seg_len; ++i)
	{
		strbuff[i] = buffer[seg_addr + i];
	}
	strbuff[seg_len] = '\0';
	gga_data->alt = atof(strbuff);

	// Undulation
	get_segment_address_and_length(buffer, len, ',', 11, &seg_addr, &seg_len);
	for(uint16_t i = 0; i < seg_len; ++i)
	{
		strbuff[i] = buffer[seg_addr + i];
	}
	strbuff[seg_len] = '\0';
	gga_data->undulation = atof(strbuff);

	return 0;
}

