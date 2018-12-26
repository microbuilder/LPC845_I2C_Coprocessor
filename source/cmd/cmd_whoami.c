/*
 * cmd_whoami.c
 *
 *  Created on: Nov 25, 2018
 *      Author: kevin
 */

#include <cmd/cmd_whoami.h>
#include <stdio.h>
#include "board.h"

/**
 * This command provides an 8-bit ID to help identify the device after
 * I2C bus scans. It can be used to make sure that the sensor responding
 * to the I2C address is actually what you thing it is.
 */
#define CMD_WHOAMI_ID	(0x45)

static struct cmd_record cmd_whoami_record = {
	.op_code = CMD_OP_CODE_WHOAMI,
	.write = &cmd_whoami_write_cb,
	.read = &cmd_whoami_read_cb,
	.timeout = 0
};

int
cmd_whoami_write_cb(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
	/* This register is read only, ignore any write attempts. */

	return kStatus_Success;
}

int
cmd_whoami_read_cb(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
	if (pdata->len) {
		memset(pdata->payload, 0, pdata->len);
		pdata->payload[0] = CMD_WHOAMI_ID;
	}

	return kStatus_Success;
}

int
cmd_whoami_init(void)
{
	int rc;

	/* Register cmd with the command registry */
	rc = cmd_register(&cmd_whoami_record);
	if (rc) {
		goto err;
	}

	return kStatus_Success;
err:
	return rc;
}
