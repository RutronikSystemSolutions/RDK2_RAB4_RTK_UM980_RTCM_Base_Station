/*
 * gga_packet.h
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

#ifndef UM980_GGA_PACKET_H_
#define UM980_GGA_PACKET_H_

#include <stdint.h>

typedef struct
{
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t sub_seconds;
	double lat_degree;
	double lat_seconds;
	uint8_t lat_dir;
	double lon_degree;
	double lon_seconds;
	uint8_t lon_dir;
	uint8_t quality;
	uint8_t satellites_in_use;
	double hdop; /**< Horizontal dilution of precision - the smaller the value, the better the quality */
	double alt; /**< Altitude above/below MSL (geoid) */
	double undulation;	/**< Geoidal separation, the difference between the Earth ellipsoid surface and mean-sealevel (geoid) surface.
							If the geoid is above the ellipsoid, the value is positive; otherwise, it is negative. */
} um980_gga_packet_t;

/**
 * @brief Given a valid NMEA packet, extract the GGA data
 *
 * @param [in] buffer Buffer containing the raw data string. Example: $GNGGA,023634.00,4004.73871635,N,11614.19729418,E,1,28,0.7,61.0988,M,-8.4923,M,,*58
 * @param [in] len Length of the buffer
 * @param [out] gga_data Pointer to a structure that will be filled
 *
 * @retval 0 Success
 * @retval -1 Cannot extract information (unvalid packet?)
 */
int gga_packet_extract_data(uint8_t* buffer, uint16_t len, um980_gga_packet_t* gga_data);


#endif /* UM980_GGA_PACKET_H_ */
