/*
 * packet_printer.h
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

#ifndef UM980_PACKET_PRINTER_H_
#define UM980_PACKET_PRINTER_H_


#include "gga_packet.h"

void packet_printer_print_gga(um980_gga_packet_t* packet);

void packet_printer_print_rtcm(uint8_t* buffer, uint16_t len);

#endif /* UM980_PACKET_PRINTER_H_ */
