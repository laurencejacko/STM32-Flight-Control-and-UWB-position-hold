/*! ----------------------------------------------------------------------------
 *  @file    read_dev_id.c
 *  @brief   This example just read DW IC's device ID. It can be used to verify
 *           the SPI comms are working correctly.
 *
 * @attention
 *
 * Copyright 2018 - 2021 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "deca_probe_interface.h"
#include "deca_device_api.h"
#include <example_selection.h>
#include <port.h>
#include <stdio.h>

#include "config_options.h"

extern SPI_HandleTypeDef hspi1;
#if defined(TEST_READING_DEV_ID)

extern void test_run_info(unsigned char *data);

/* Example application name and version to display on LCD screen/VCOM port. */
#define APP_NAME "READ DEV ID      "

/**
 * Application entry point.
 */
extern dwt_config_t config_options;


int read_dev_id(void)
{
    int err;
    uint32_t dev_id;

    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 36 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)


   /* uint8_t txbuf[5];

    txbuf[0] = 0x00;
    txbuf[1] = 0x00;
    txbuf[2] = 0x00;
    txbuf[3] = 0x00;
    txbuf[4] = 0x00;
    uint8_t rxbuf[4];

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, txbuf,5, 1000);
    HAL_SPI_Receive(&hspi1, rxbuf,4, 1000);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);
*/


    /* Probe for the correct device driver. */
    int value = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    dev_id = dwt_readdevid();

    /* Reads and validate device ID returns DWT_ERROR if it does not match expected else DWT_SUCCESS */
    if ((err = dwt_check_dev_id()) == DWT_SUCCESS)
    {
        test_run_info((unsigned char *)"DEV ID OK");
    }
    else
    {
        test_run_info((unsigned char *)"DEV ID FAILED");
    }


    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

       if (dwt_initialise(0x0) == DWT_ERROR)
       {
           test_run_info((unsigned char *)"INIT FAILED     ");
           while (1) { };
       }
       int res = dwt_configure(&config_options);




    return err;
}

#endif
/*****************************************************************************************************************************************************
 * NOTES:
 ****************************************************************************************************************************************************/
