/*
 * cmd_version.c
 *
 *  Created on: Nov 30, 2018
 *      Author: kevin
 */

#include <stdio.h>
#include "board.h"
#include "cmd_version.h"

/**
 * This command can be used to indicate the firmware version of
 * the I2C co-processor in cases where behaviour may change over
 * device releases.
 */
#define CMD_VERSION_REVISION	(0x0A)

static struct cmd_record cmd_version_record = {
	.op_code = CMD_OP_CODE_VERSION,
	.write = &cmd_version_write_cb,
	.read = &cmd_version_read_cb,
	.timeout = 0
};

int
cmd_version_write_cb(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
	/* This register is read only, ignore any write attempts. */

	return kStatus_Success;
}

int
cmd_version_read_cb(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
	if (pdata->len) {
		memset(pdata->payload, 0, pdata->len);
		pdata->payload[0] = CMD_VERSION_REVISION;
	}

	return kStatus_Success;
}

int
cmd_version_init(void)
{
	int rc;

	/* Register cmd with the command registry */
	rc = cmd_register(&cmd_version_record);
	if (rc) {
		goto err;
	}

	return kStatus_Success;
err:
	return rc;
}
