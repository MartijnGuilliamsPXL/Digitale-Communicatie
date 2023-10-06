/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
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
*******************************************************************************/

/* Project Description
 *
 * In this project, the SMART-IO Block has been configured in the PDL which
 * does the following:
 * (1) Takes the PWM outputs as input on Chip1 and inverts it into I/O0 (P9[0])
 * (2) Takes the LUT1,2,4 as inputs and outputs it into P9[1], P[2] and P9[4] respectively
 *
 * To observe the outputs, the USER LED and RGB Leds are used. The outputs of the SMART-IO are
 * fed into pins 5[0] to 5[3]. The value on these pins are read and written into USER LED and
 * the RGB LEDs.
 *
 * Connections:
 * 		P9[0] ---> P5[0]
 * 		P9[1] ---> P5[1]
 * 		P9[2] ---> P5[2]
 * 		P9[4] ---> P5[3]
 *
 * Output: You should see USER LED blinking every 1s and RGB LEDs changing color in this fashion:
 * 		OFF --> RED --> GREEN --> YELLOW --> BLUE --> PINK --> INDIGO --> WHITE --> OFF
 *
 * 		Note: For PSoC6 Proto Kit, you will only see the output
 *        in USER LED since it doesn't have RGB.
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_syslib.h"

/* Guard for Proto Kit */

#ifndef CYBSP_LED_RGB_BLUE
	#define EXTERN (P13_6)
#endif


int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    // Initialize all the LED Pins
    cyhal_gpio_init(P13_6, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    cyhal_gpio_write(P13_6, 0);
    Cy_SysLib_Delay(5000);

    for (;;)
    {

    	cyhal_gpio_write(P13_6, 1);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 1);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 1);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 1);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 1);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
    	Cy_SysLib_Delay(500);
    	cyhal_gpio_write(P13_6, 0);
		Cy_SysLib_Delay(500);
		cyhal_gpio_write(P13_6, 0);
		Cy_SysLib_Delay(500);
		cyhal_gpio_write(P13_6, 0);
		Cy_SysLib_Delay(500);
		cyhal_gpio_write(P13_6, 0);
		Cy_SysLib_Delay(500);
		cyhal_gpio_write(P13_6, 0);
		Cy_SysLib_Delay(500);
		cyhal_gpio_write(P13_6, 0);
		Cy_SysLib_Delay(500);
		cyhal_gpio_write(P13_6, 0);
		Cy_SysLib_Delay(500);
		cyhal_gpio_write(P13_6, 0);
		Cy_SysLib_Delay(500);
		cyhal_gpio_write(P13_6, 0);
		Cy_SysLib_Delay(500);


    }
}

/* [] END OF FILE */
