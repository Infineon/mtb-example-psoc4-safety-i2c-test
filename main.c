/*******************************************************************************
* File Name:   main.c
*
* Description: This file provides example usage of I2C self tests for PSoC 4.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
********************************************************************************/

/*******************************************************************************
* Includes
********************************************************************************/
#include <stdio.h>
#include "cy_pdl.h"
#include "cybsp.h"
#include "SelfTest_I2C_SCB.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define MAX_INDEX_VAL (0xFFF0u)
#define I2C_TEST_RESULT "I2C Protocol communication test at run-time..."
#define LED_DELAY_MS 50

/*******************************************************************************
* Global Variables
********************************************************************************/
static cy_stc_scb_i2c_context_t CYBSP_DUT_I2C_MASTER_context;
static cy_stc_scb_i2c_context_t CYBSP_DUT_I2C_SLAVE_context;
static uint8_t i2c_slave_read_buf[PACKET_SIZE];
static uint8_t i2c_slave_write_buf[PACKET_SIZE];
char uart_print_buff[100]={0};

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void CYBSP_DUT_I2C_MASTER_Interrupt(void);
static void CYBSP_DUT_I2C_SLAVE_Interrupt(void);
static void SelfTest_I2C_SCB_Init(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* The main function performs the following tasks:
*    1. Initializes the device, board peripherals, and retarget-io for prints.
*    2. Calls the SelfTest_I2C_SCB_Init function to initialize the UART component.
*    3. Calls the SelfTest_I2C_SCB API to test the I2C SCB IP.
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
    cy_rslt_t result;
    cy_stc_scb_uart_context_t CYBSP_UART_context;

    uint8_t ret;

    uint16_t count = 0u;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\nClass-B Safety Test: I2C\r\n");

    /* Init I2C SelfTest*/
    SelfTest_I2C_SCB_Init();

    for (;;)
    {
        /*******************************/
        /* Run I2C Self Test...       */
        /*******************************/
        ret = SelfTest_I2C_SCB(CYBSP_DUT_I2C_MASTER_HW, &CYBSP_DUT_I2C_MASTER_context,
                CYBSP_DUT_I2C_SLAVE_HW, &CYBSP_DUT_I2C_SLAVE_context,
                i2c_slave_read_buf, i2c_slave_write_buf);
        if ((PASS_COMPLETE_STATUS != ret) && (PASS_STILL_TESTING_STATUS != ret))
        {
            /* Process error */
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\nI2C Protocol test: error \r\n");
            /* Set the user LED state */
            Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

            /* In case of error detection message is printed to DEBUG Output and the user
             * LED is set. Also the code execution is stopped here in a while loop */
            while (1)
            {
            }
        }

        /* Print test counter */
        sprintf(uart_print_buff, "\r%s count=%d", I2C_TEST_RESULT, count);
        Cy_SCB_UART_PutString(CYBSP_UART_HW, uart_print_buff);

        /* Toggle the user LED state */
        Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

        /* Wait for 0.05 seconds */
        Cy_SysLib_Delay(LED_DELAY_MS);

        count++;
        if (count > MAX_INDEX_VAL)
        {
            count = 0u;
        }
    }
}

/*****************************************************************************
* Function Name: SelfTest_I2C_SCB_Init
******************************************************************************
*
* Summary:
*  Init I2C components and create I2C slave buffers
*
* Parameters:
*  NONE
*
* Return:
*  NONE
*
* Note:
*
*****************************************************************************/
static void SelfTest_I2C_SCB_Init(void)
{
    cy_en_scb_i2c_status_t i2c_res;
    cy_en_sysint_status_t int_res;
    
    const cy_stc_sysint_t CYBSP_DUT_I2C_MASTER_IRQ_config =
    {
            /*.intrSrc =*/ CYBSP_DUT_I2C_MASTER_IRQ,
            /*.intrPriority =*/ 3u
    };

    /*Initialize and enable the I2C in master mode*/
    i2c_res = Cy_SCB_I2C_Init(CYBSP_DUT_I2C_MASTER_HW, &CYBSP_DUT_I2C_MASTER_config, &CYBSP_DUT_I2C_MASTER_context);
    if(i2c_res != CY_SCB_I2C_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Hook interrupt service routine */
    int_res = Cy_SysInt_Init(&CYBSP_DUT_I2C_MASTER_IRQ_config, &CYBSP_DUT_I2C_MASTER_Interrupt);
    if(int_res != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    NVIC_EnableIRQ((IRQn_Type) CYBSP_DUT_I2C_MASTER_IRQ_config.intrSrc);
    Cy_SCB_I2C_Enable(CYBSP_DUT_I2C_MASTER_HW, &CYBSP_DUT_I2C_MASTER_context);

    /* Initialize and enable I2C Component in slave mode. */

    /* CYBSP_I2C_SCB_IRQ */
    const cy_stc_sysint_t CYBSP_DUT_I2C_SLAVE_IRQ_config = {
            .intrSrc = (IRQn_Type) CYBSP_DUT_I2C_SLAVE_IRQ,
            .intrPriority = 3u,
    };

    i2c_res = Cy_SCB_I2C_Init(CYBSP_DUT_I2C_SLAVE_HW, &CYBSP_DUT_I2C_SLAVE_config, &CYBSP_DUT_I2C_SLAVE_context);
    if(i2c_res != CY_SCB_I2C_SUCCESS)
    {
        CY_ASSERT(0);
    }

    int_res = Cy_SysInt_Init(&CYBSP_DUT_I2C_SLAVE_IRQ_config, &CYBSP_DUT_I2C_SLAVE_Interrupt);
    if(int_res != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure read buffer */
    Cy_SCB_I2C_SlaveConfigReadBuf(CYBSP_DUT_I2C_SLAVE_HW, i2c_slave_read_buf,
            PACKET_SIZE, &CYBSP_DUT_I2C_SLAVE_context);

    /* Configure write buffer */
    Cy_SCB_I2C_SlaveConfigWriteBuf(CYBSP_DUT_I2C_SLAVE_HW, i2c_slave_write_buf,
            PACKET_SIZE, &CYBSP_DUT_I2C_SLAVE_context);

    /*  Enable interrupt and I2C block */
    NVIC_EnableIRQ((IRQn_Type) CYBSP_DUT_I2C_SLAVE_IRQ_config.intrSrc);

    Cy_SCB_I2C_Enable(CYBSP_DUT_I2C_SLAVE_HW, &CYBSP_DUT_I2C_SLAVE_context);
}


/*******************************************************************************
* Function Name: CYBSP_DUT_I2C_MASTER_Interrupt
****************************************************************************//**
*
* Summary:
*   Invokes the Cy_SCB_I2C_Interrupt() PDL driver function.
*
*******************************************************************************/
static void CYBSP_DUT_I2C_MASTER_Interrupt(void)
{
    Cy_SCB_I2C_Interrupt(CYBSP_DUT_I2C_MASTER_HW, &CYBSP_DUT_I2C_MASTER_context);
}


/*******************************************************************************
* Function Name: CYBSP_DUT_I2C_SLAVE_Interrupt
****************************************************************************//**
*
* Summary:
*   Invokes the Cy_SCB_I2C_Interrupt() PDL driver function.
*
*******************************************************************************/
static void CYBSP_DUT_I2C_SLAVE_Interrupt(void)
{
    Cy_SCB_I2C_Interrupt(CYBSP_DUT_I2C_SLAVE_HW, &CYBSP_DUT_I2C_SLAVE_context);
}

/* [] END OF FILE */
