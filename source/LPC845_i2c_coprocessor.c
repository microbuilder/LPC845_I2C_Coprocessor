/*
 * The Clear BSD License
 * Copyright 2018 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* On the DIP-format LPC845 Breakout Board, I2C is setup for the following pins:
 *
 *   o I2C0_SCL = P0_10 = 'DIP' Pin 23
 *   o I2C0_SDA = P0_11 = 'DIP' Pin 24
 *
 *   If no pullup resistors are present on the I2C master side, SCL0 and SDA0
 *   should be pulled high (to VDD) using ~2.2K resistors.
 */

#include <stdio.h>
#include <string.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "pin_mux.h"

#include "i2c_cfg.h"
#include "cmd/cmd.h"

#define I2C_SLAVE ((I2C_Type *)I2C_CFG_SLAVE_BASE)

uint8_t g_slave_tx_buff[I2C_CFG_MAX_DATA_LENGTH] = { 0 };
uint8_t g_slave_rx_buff[I2C_CFG_MAX_DATA_LENGTH] = { 0 };

i2c_slave_handle_t g_s_handle;

volatile bool g_i2cs_start_event = false;
volatile bool g_i2cs_write_event = false;
volatile bool g_i2cs_completed = false;
volatile uint8_t g_i2cs_last_reg_write = 0xFFU;

/**
 * Debug stats for I2C slave events.
 */
volatile struct {
	uint32_t events;		/**< Number of times the callback handler fired. */
	uint32_t addr_matches;	/**< Number of times an address match occurred. */
	uint32_t transmits;		/**< Number of I2C transmit events. */
	uint32_t receives;		/**< Number of I2C receive events. */
	uint32_t stops;			/**< Number of STOP events received. */
	uint32_t unhandled;		/**< Number of unhandled events. */
} g_i2cs_stats = { 0 };

/**
 * Callback handler for events on the I2C slave bus.
 */
static void i2c_slave_callback(I2C_Type *base, volatile i2c_slave_transfer_t *xfer, void *userData)
{
	struct cmd_data pdata;
	g_i2cs_stats.events++;

    switch (xfer->event)
    {
        /* Slave address match after a start or repeated start */
        case kI2C_SlaveAddressMatchEvent:
        	g_i2cs_start_event = true;
            xfer->rxData = NULL;
            xfer->rxSize = 0;
            g_i2cs_stats.addr_matches++;
            break;

		/* Callback is requested to provide a buffer in which to place received data (slave-receiver role) */
		case kI2C_SlaveReceiveEvent:
			g_i2cs_write_event = true;
			/*  Update information for received process */
			xfer->rxData = g_slave_rx_buff;
			xfer->rxSize = 1;
			g_i2cs_stats.receives++;
			break;

		/* All data in the active transfer have been consumed. Normally the master has sent a stop transition on the bus */
		case kI2C_SlaveCompletionEvent:
			g_i2cs_completed = true;
			g_i2cs_stats.stops++;
			break;

        /* Callback is requested to provide data to transmit (slave-transmitter role). */
        case kI2C_SlaveTransmitEvent:
        	pdata.len = 1;
        	pdata.payload = g_slave_tx_buff;
        	status_t reVal = cmd_read(g_i2cs_last_reg_write, &pdata, NULL);
        	if (reVal == kStatus_InvalidArgument) {
        		/* Probably an unsupported opcode, set the value to 0xFF by default. */
        		g_slave_tx_buff[0] = 0xFF;
        	}
            /*  Update information for transmit process */
            xfer->txData = g_slave_tx_buff;
            xfer->txSize = 1;
            g_i2cs_stats.transmits++;
            break;

        default:
            g_i2cs_completed = false;
            g_i2cs_stats.unhandled++;
            break;
    }
}

/**
 * Initialises the I2C bus in slave mode using a 7-bit address
 * and settings defined in i2c_cfg.h.
 */
int init_i2c_slave(i2c_slave_config_t *slaveConfig)
{
    status_t reVal = kStatus_Fail;

    /* Set up i2c slave */
    I2C_SlaveGetDefaultConfig(slaveConfig);

    /* Change the slave address */
    slaveConfig->address0.address = I2C_CFG_ADDR_7BIT;

    /* Initialize the I2C slave peripheral */
    I2C_SlaveInit(I2C_SLAVE, slaveConfig, I2C_CFG_CLOCK_FREQUENCY);

    memset(g_slave_tx_buff, 0, sizeof(g_slave_tx_buff));
    memset(g_slave_rx_buff, 0, sizeof(g_slave_rx_buff));

    /* Create the I2C handle for the non-blocking transfer */
    I2C_SlaveTransferCreateHandle(I2C_SLAVE, &g_s_handle, i2c_slave_callback, NULL);

    /* Start accepting I2C transfers on the I2C slave peripheral */
    reVal = I2C_SlaveTransferNonBlocking(I2C_SLAVE, &g_s_handle,
    		                             kI2C_SlaveCompletionEvent | kI2C_SlaveAddressMatchEvent);

    return reVal;
}

/**
 * This loop waits for incoming I2CS requests, and processes any read/write
 * requests in an appropriate order.
 */
int handle_i2cs_xfer(void)
{
    status_t reVal = kStatus_Success;
	struct cmd_data pdata;

    /* Don't start processing until we get a new I2C start or repeated start event */
    while(g_i2cs_start_event == false) {
    	__asm volatile("nop");
    }
    g_i2cs_start_event = false;

	/* Wait for the end of the message. */
    while (!g_i2cs_completed)
    {
    	__asm volatile("nop");
    }
    g_i2cs_completed = false;

    /* Handle write requests (data master->slave) */
    if (g_i2cs_write_event) {
    	/* Track the last register we wrote to */
    	g_i2cs_last_reg_write = g_slave_rx_buff[0];
    	/* Send the write request to the appropriate command handler. */
    	pdata.len = g_slave_rx_buff[1];
    	pdata.payload = &g_slave_rx_buff[2];
    	reVal = cmd_write(g_i2cs_last_reg_write, &pdata, NULL);
    	if (reVal == kStatus_InvalidArgument) {
    		PRINTF("Unsupported opcode: 0x%02X\n", g_i2cs_last_reg_write);
    	}
    	g_i2cs_write_event = false;
    }

    return reVal;
}

/*!
 * @brief Main function
 */
int main(void)
{
    i2c_slave_config_t slaveConfig;
    status_t reVal = kStatus_Fail;

    /* Select the main clock as source clock of USART0 (debug console) */
    CLOCK_Select(BOARD_DEBUG_USART_CLK_ATTACH);

    /* Select the main clock as source clock of I2C0. */
    CLOCK_Select(kI2C0_Clk_From_MainClk);

    BOARD_InitPins();
    BOARD_BootClockFRO30M();
    BOARD_InitDebugConsole();

    LED_GREEN_INIT(1);
    LED_BLUE_INIT(1);
    LED_RED_INIT(1);
    LED_RED_ON();

    PRINTF("\nLPC845 I2C co-processor\n");

    /* Init the command parser */
    PRINTF("Starting the command parsing engine\n");
	reVal = cmd_init();
    if (reVal != kStatus_Success) {
        PRINTF("Unable to init command parser!");
        while(1) {
        	__asm volatile("nop");
        }
	}

    /* Init I2C slave */
    reVal = init_i2c_slave(&slaveConfig);
    if (reVal != kStatus_Success) {
        PRINTF("Unable to init I2C slave!");
        while(1) {
        	__asm volatile("nop");
        }
    }

    /* Display the I2C slave address for convenience sake. */
    PRINTF("I2C address = 0x%2X\n", I2C_CFG_ADDR_7BIT);

    /* Visual indication of ready status */
    LED_RED_OFF();
    LED_BLUE_ON();

    /* Process incoming I2C requests. */
    while (1) {
    	reVal = handle_i2cs_xfer();
    	if (reVal != kStatus_Success) {
    		PRINTF("I2C slave error\n");
    	}
    }
}
