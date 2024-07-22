/*
 * strutils.h
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

#ifndef UM980_STRUTILS_H_
#define UM980_STRUTILS_H_

#include <stdint.h>

/**
 * @brief Given a string, a delimiter and a segment index, return a pointer on the segment and its length
 *
 * @param [in] buffer String to be parsed
 * @param [in] len Length of the buffer
 * @param [in] delim Delimiter, example: ','
 * @param [in] index Index of the segment we want to identify
 * @param [out] seg_addr Index where the selected segment starts (relative to buffer)
 * @param [out] seg_len Length of the selected segment
 *
 * @retval 0 Success
 * @retval -1 Cannot find the segment
 */
int get_segment_address_and_length(uint8_t* buffer, uint16_t len, uint8_t delim, uint16_t index, uint16_t* seg_addr, uint16_t* seg_len);

/**
 * @brief Given a string and a delimiter, check how much segment the string contains
 *
 * @param [in] buffer String to be parsed
 * @param [in] len Length of the buffer
 * @param [in] delim Delimiter, example: ','
 *
 * @retval Number of segments inside the string
 */
int get_segment_count(uint8_t* buffer, uint16_t len, uint8_t delim);

#endif /* UM980_STRUTILS_H_ */
