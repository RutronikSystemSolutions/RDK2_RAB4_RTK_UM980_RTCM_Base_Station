/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK2_UM980_NTRIP_Client
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*  Created on: 2023-10-18
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Industriestraße 2, 75228 Ispringen, Allemagne
*  Author: ROJ030
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cy_usb_dev.h"
#include "cycfg_usbdev.h"
#include "cy_retarget_io.h"

#include "usb_transfer.h"

#include "um980/um980_app.h"
#include "um980/packet_printer.h"
#include "um980/nmea_packet.h"
#include "hal/hal_uart.h"
#include "hal/hal_timer.h"

/**
 * @def DYNAMIC_BASE_POSITION
 *
 * If defined, the position of the base station is computed dynamically (during the first 60 seconds)
 * If not defined, you have to give the position of the base station (directly in the source code below)
 */
#define DYNAMIC_BASE_POSITION

/**
 * Some defines useful for USB transfer
 */
#define USBUART_COM_PORT		(0U)
#define USB_TRANSFER_BUF_SIZE	1024

static void usb_high_isr(void);
static void usb_medium_isr(void);
static void usb_low_isr(void);
void handle_error(void);

/* USB Interrupt Configuration */
const cy_stc_sysint_t usb_high_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_hi_IRQn,
    .intrPriority = 5U,
};
const cy_stc_sysint_t usb_medium_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_med_IRQn,
    .intrPriority = 6U,
};
const cy_stc_sysint_t usb_low_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_lo_IRQn,
    .intrPriority = 7U,
};

/* USBDEV context variables */
cy_stc_usbfs_dev_drv_context_t  usb_drvContext;
cy_stc_usb_dev_context_t        usb_devContext;
cy_stc_usb_dev_cdc_context_t    usb_cdcContext;

static uint8_t usb_transfer_is_ready()
{
	return Cy_USB_Dev_CDC_IsReady(USBUART_COM_PORT, &usb_cdcContext);
}

static int usb_transfer_put_data(uint8_t* buffer, uint16_t size)
{
	return Cy_USB_Dev_CDC_PutData(USBUART_COM_PORT, buffer, size, &usb_cdcContext);
}

/**
 * @brief Listener function that will be called when a new NMEA packet is available
 *
 * A typical NMEA packet is a GGA packet containing the position
 */
void um980_nmea_listener(uint8_t* buffer, uint16_t len)
{
	if (nmea_packet_get_type(buffer, len) == PACKET_TYPE_GGA)
	{
		um980_gga_packet_t gga_data;
		if (gga_packet_extract_data(buffer, len, &gga_data) == 0)
		{
			packet_printer_print_gga(&gga_data);
		}
	}
}

/**
 * @brief Listener function that will be called when a new RTCM packet is available
 *
 * RTCM packet contains correction data to be used to enter RTK mode
 */
void um980_rtcm_listener(uint8_t* buffer, uint16_t len)
{
	// Print on UART
	packet_printer_print_rtcm(buffer, len);

	// Send over USB the raw data
	usb_transfer_send(buffer, len);

	// signal something happened
	cyhal_gpio_toggle(LED1);
}

void do_blink()
{
	static const uint32_t timeout = 500000; // 0.5 seconds
	static uint32_t last_toggle = 0;
	uint32_t current = hal_timer_get_uticks();

	if ((current < last_toggle) || ((current - last_toggle) > timeout))
	{
		last_toggle = current;
		cyhal_gpio_toggle(LED2);
	}
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function for CM4 CPU. It initializes the USB Device block
*  and enumerates as a CDC device. It constantly checks for data received from
*  host and echos it back.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    // Initialize the device and board peripherals
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    // Initialize the retarget-io (for printf)
	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
								  CY_RETARGET_IO_BAUDRATE);

	printf("Start RDK2_RAB4_RTK_UM980_RTCM_Base_Station example.\r\n");

    // Enable global interrupts
    __enable_irq();

    // Initialize LEDs
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    // PVT input
    result = cyhal_gpio_init(ARDU_IO3, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    // Initialize UART
    if (hal_uart_init() != 0)
    {
    	handle_error();
    }
    printf("UART initialized.\r\n");

	int retval = hal_timer_init();
	if (retval != 0)
	{
		printf("Error while initializing timer. \r\n");
		for(;;){}
	}
	printf("Timer initialized.\r\n");

	// Initialize UM980
	um980_app_init_hal(hal_uart_readable, hal_uart_read, hal_uart_write, hal_timer_get_uticks);
	um980_app_set_nmea_listener(um980_nmea_listener);
	um980_app_set_rtcm_listener(um980_rtcm_listener);

	// Stop message generation (correction and position)
	// Wait until successful - First call might fail because of strange startup behavior
	for(;;)
	{
		retval = um980_app_unlog();
		if (retval != 0)
		{
			printf("um980_app_unlog error %d ! \r\n", retval);
			Cy_SysLib_Delay(1000);
			um980_app_reset();
		}
		else break;
	}

	// Initialize the USB device
	Cy_USB_Dev_Init(USB_DEV_HW, &USB_DEV_config, &usb_drvContext,
					&usb_devices[0], &usb_devConfig, &usb_devContext);

	// Initialize the CDC Class
	Cy_USB_Dev_CDC_Init(&usb_cdcConfig, &usb_cdcContext, &usb_devContext);

	// Initialize the USB interrupts
	Cy_SysInt_Init(&usb_high_interrupt_cfg,   &usb_high_isr);
	Cy_SysInt_Init(&usb_medium_interrupt_cfg, &usb_medium_isr);
	Cy_SysInt_Init(&usb_low_interrupt_cfg,    &usb_low_isr);

	// Enable the USB interrupts
	NVIC_EnableIRQ(usb_high_interrupt_cfg.intrSrc);
	NVIC_EnableIRQ(usb_medium_interrupt_cfg.intrSrc);
	NVIC_EnableIRQ(usb_low_interrupt_cfg.intrSrc);

	// Make device appear on the bus. This function call is blocking, it waits till the device enumerates
	Cy_USB_Dev_Connect(true, CY_USB_DEV_WAIT_FOREVER, &usb_devContext);

	printf("Connected over USB. \r\n");

	// Initialize the transfer module (enabling to send packet over USB)
	usb_transfer_init(USB_TRANSFER_BUF_SIZE, usb_transfer_is_ready, usb_transfer_put_data);

#ifdef DYNAMIC_BASE_POSITION
	// Set base mode
	if (um980_app_set_mode_base() != 0)
	{
		printf("um980_app_set_mode_base error ! \r\n");
		handle_error();
	}
#else
	// Coordinates of Rutronik Ispringen -> customize it!
	double latitude = 48.91221729972876;
	double longitude = 8.666026440355095;
	double altitude = 275.0;

	retval = um980_app_set_mode_base_with_position(latitude, longitude, altitude);
	if (retval != 0)
	{
		printf("Error by um980_app_set_mode_base_with_position -> %d \r\n", retval);
		handle_error();
	}
#endif

	if (um980_app_start_correction_generation(1006, 10) != 0)
	{
		printf("um980_app_start_correction_generation error ! \r\n");
		handle_error();
	}

	if (um980_app_start_correction_generation(1033, 10) != 0)
	{
		printf("um980_app_start_correction_generation error ! \r\n");
		handle_error();
	}

	if (um980_app_start_correction_generation(1074, 1) != 0)
	{
		printf("um980_app_start_correction_generation error ! \r\n");
		handle_error();
	}

	if (um980_app_start_correction_generation(1084, 1) != 0)
	{
		printf("um980_app_start_correction_generation error ! \r\n");
		handle_error();
	}

	if (um980_app_start_correction_generation(1094, 1) != 0)
	{
		printf("um980_app_start_correction_generation error ! \r\n");
		handle_error();
	}

	if (um980_app_start_correction_generation(1124, 1) != 0)
	{
		printf("um980_app_start_correction_generation error ! \r\n");
		handle_error();
	}

	if (um980_app_start_correction_generation(1114, 1) != 0)
	{
		printf("um980_app_start_correction_generation error ! \r\n");
		handle_error();
	}

	// Request position every second
	if (um980_app_start_gga_generation(2) != 0)
	{
		printf("um980_app_start_gga_generation error ! \r\n");
		handle_error();
	}

	printf("UM980 initialized. Start to generate data. \r\n");

	for(;;)
	{
		if (um980_app_do() != 0)
		{
			printf("Error app_do()\r\n");
			handle_error();
		}

		usb_transfer_do();

		do_blink();
	}
}

/***************************************************************************
* Function Name: usb_high_isr
********************************************************************************
* Summary:
*  This function processes the high priority USB interrupts.
*
***************************************************************************/
static void usb_high_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USB_DEV_HW, Cy_USBFS_Dev_Drv_GetInterruptCauseHi(USB_DEV_HW),
                               &usb_drvContext);
}


/***************************************************************************
* Function Name: usb_medium_isr
********************************************************************************
* Summary:
*  This function processes the medium priority USB interrupts.
*
***************************************************************************/
static void usb_medium_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USB_DEV_HW, Cy_USBFS_Dev_Drv_GetInterruptCauseMed(USB_DEV_HW),
                               &usb_drvContext);
}


/***************************************************************************
* Function Name: usb_low_isr
********************************************************************************
* Summary:
*  This function processes the low priority USB interrupts.
*
**************************************************************************/
static void usb_low_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USB_DEV_HW, Cy_USBFS_Dev_Drv_GetInterruptCauseLo(USB_DEV_HW),
                               &usb_drvContext);
}

void handle_error(void)
{
	printf("Something wrong happened. \r\n");

	// both LEDs OFF
	cyhal_gpio_write(LED1, CYBSP_LED_STATE_OFF);
	cyhal_gpio_write(LED2, CYBSP_LED_STATE_OFF);

     /* Disable all interrupts. */
    __disable_irq();
    CY_ASSERT(0);
}

/* [] END OF FILE */
