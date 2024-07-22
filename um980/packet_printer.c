/*
 * packet_printer.c
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

#include "packet_printer.h"

#include <stdio.h>
#include "rtcm_packet.h"
#include <string.h>

void packet_printer_print_gga(um980_gga_packet_t* packet)
{
	double real_lat = packet->lat_degree + packet->lat_seconds / (double)60;
	if (packet->lat_dir == 1) real_lat = -real_lat;

	double real_lon = packet->lon_degree + packet->lon_seconds / (double)60;
	if (packet->lon_dir == 1) real_lon = -real_lon;

	printf("------------ GGA packet ------------\r\n");
	printf("UTC: %2d:%2d:%2d \r\n", packet->hours, packet->minutes, packet->seconds);
	printf("Longitude: %f deg.\r\n", (float)real_lon);
	printf("Latitude:  %f deg.\r\n", (float)real_lat);
	printf("Quality:  %d \r\n", packet->quality);
	printf("Satellites in use:  %d\r\n", packet->satellites_in_use);
	printf("Horizontal dilution of precision:  %f\r\n", (float)packet->hdop);

	double real_alt = packet->alt + packet->undulation;
	printf("Altitude:  %f m.\r\n", (float)real_alt);
}

void packet_printer_print_rtcm(uint8_t* buffer, uint16_t len)
{
	uint16_t type = rtcm_packet_get_type(buffer);

	printf("------------ RTCM packet ------------\r\n");
	if (len < 5) return;
	printf("Type : %d \t Data len : %d \r\n", type, rtcm_packet_get_variable_size(buffer));

//	if (type == 1033)
//	{
//		uint8_t str[32] = {0};
//
//		// Extract the addresses of the field (variable packet)
//		uint8_t antenna_descr_len = buffer[6];
//		uint8_t antenna_descr_addr = 7;
//		uint8_t antenna_setup_id_addr = antenna_descr_addr + antenna_descr_len;
//		uint8_t antenna_serial_nb_len_addr =  antenna_setup_id_addr + 1;
//		uint8_t antenna_serial_nb_len = buffer[antenna_serial_nb_len_addr];
//		uint8_t antenna_serial_nb_addr = antenna_serial_nb_len_addr + 1;
//		uint8_t receiver_type_len_addr = antenna_serial_nb_addr + antenna_serial_nb_len;
//		uint8_t receiver_type_len = buffer[receiver_type_len_addr];
//		uint8_t receiver_type_addr = receiver_type_len_addr + 1;
//		uint8_t receiver_fw_version_len_addr = receiver_type_addr + receiver_type_len;
//		uint8_t receiver_fw_version_len = buffer[receiver_fw_version_len_addr];
//		uint8_t receiver_fw_version_addr = receiver_fw_version_len_addr + 1;
//		uint8_t receiver_serial_nb_len_addr = receiver_fw_version_addr + receiver_fw_version_len;
//		uint8_t receiver_serial_nb_len = buffer[receiver_serial_nb_len_addr];
//		uint8_t receiver_serial_nb_addr = receiver_serial_nb_len_addr + 1;
//
//
//		if (antenna_descr_len > 31) return;
//		if (antenna_serial_nb_len > 31) return;
//		if (receiver_type_len > 31) return;
//		if (receiver_fw_version_len > 31) return;
//		if (receiver_serial_nb_len > 31) return;
//
//		memcpy(str, &buffer[antenna_descr_addr], antenna_descr_len);
//		str[antenna_descr_len] = '\0';
//		printf("Antenna descriptor : %s \r\n", str);
//
//		memcpy(str, &buffer[antenna_serial_nb_addr], antenna_serial_nb_len);
//		str[antenna_serial_nb_len] = '\0';
//		printf("Antenna serial number : %s \r\n", str);
//
//		memcpy(str, &buffer[receiver_type_addr], receiver_type_len);
//		str[receiver_type_len] = '\0';
//		printf("Receiver type : %s \r\n", str);
//
//		memcpy(str, &buffer[receiver_fw_version_addr], receiver_fw_version_len);
//		str[receiver_fw_version_len] = '\0';
//		printf("Receiver FW version : %s \r\n", str);
//
//		memcpy(str, &buffer[receiver_serial_nb_addr], receiver_serial_nb_len);
//		str[receiver_serial_nb_len] = '\0';
//		printf("Receiver serial number : %s \r\n", str);
//	}
}


