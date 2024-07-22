/*
 * usb_transfer.h
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

#ifndef USB_TRANSFER_H_
#define USB_TRANSFER_H_

#include <stdint.h>

typedef uint8_t (*usb_transfer_is_ready_func_t)(void);
typedef int (*usb_transfer_put_data_func_t)(uint8_t* buffer, uint16_t size);

/**
 * @brief Initialize the module
 *
 * @param buffer_length Length of the transfer buffer.
 *
 *
 * @retval 0 Success
 * @retval -1 Cannot allocate enough memory
 */
int usb_transfer_init(uint16_t buffer_length,
		usb_transfer_is_ready_func_t is_ready,
		usb_transfer_put_data_func_t put_data);

/**
 * @brief Send a buffer through the USB communication
 *
 * @retval 0 Success
 * @retval -1 Not enough space in the buffer
 */
int usb_transfer_send(uint8_t* buffer, uint16_t length);

/**
 * @brief Periodic call to the module to perform tasks
 *
 * Checks if something needs to be send over the USB bus and if the USB bus is ready for it
 */
void usb_transfer_do();

/**
 * @brief Check if the module has a task remaining (send data, send 0 packet, ...)
 *
 * Can be called before going to sleep for example
 *
 * @retval 0 Nothing to do
 * @retval 1 Module has something to do, call usb_transfer_do()
 */
int usb_transfer_is_task_remaining();


#endif /* USB_TRANSFER_H_ */
