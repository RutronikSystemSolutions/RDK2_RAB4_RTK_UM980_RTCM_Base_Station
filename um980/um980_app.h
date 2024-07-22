/*
 * um980_app.h
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

#ifndef UM980_UM980_APP_H_
#define UM980_UM980_APP_H_

#include <stdint.h>

#include "packet_handler.h"

typedef void (*um980_app_on_new_packet)(uint8_t* buffer, uint16_t len);

/**
 * @brief Initializes the module
 *
 * Store the HAL function and flush the RX receiving buffer
 */
void um980_app_init_hal(um980_app_uart_readable_func_t uart_readable,
		um980_app_uart_read_func_t uart_read,
		um980_app_uart_write_func_t uart_write,
		um980_app_get_uticks get_uticks);

/**
 * @brief Set callback function to be called when a new NMEA packet is available
 */
void um980_app_set_nmea_listener(um980_app_on_new_packet listener);

/**
 * @brief Set callback function to be called when a new RTCM packet is available (for base station)
 */
void um980_app_set_rtcm_listener(um980_app_on_new_packet listener);

/**
 * @brief In case of error, call this function to cleanup the internal buffer
 */
void um980_app_reset();

/**
 * @brief Make the UM980 quiet (start generation correction and position messages)
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_unlog();

/**
 * @brief Start the generation of GGA output messages (contains position, time, ...)
 *
 * @param [in] period Period in seconds
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_start_gga_generation(uint16_t period);

/**
 * @brief Set the UM980 as a base mode
 *
 * In base mode, the module first measures its position during 60seconds.
 * It can send correction data (using RTCM messages) to rover
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_set_mode_base();

/**
 * @brief Set the UM980 as a base mode. Specify its real position
 *
 * @param [in] latitude Latitude in degrees
 * @param [in] longitude Longitude in degress
 * @param [in] altitude Altitude in meters
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_set_mode_base_with_position(double latitude, double longitude, double altitude);

/**
 * @brief Set the UM980 to rover mode
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_set_mode_rover();

/**
 * @brief Tell the UM980 to generate correction messages of type rtcm_number
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_start_correction_generation(uint16_t rtcm_number, uint16_t period);


/**
 * @brief Cyclic call to be called to catch the messages sent by the UM980
 *
 * Check if a message (NMEA or RTCM) is available and read it
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_do();

#endif /* UM980_UM980_APP_H_ */
