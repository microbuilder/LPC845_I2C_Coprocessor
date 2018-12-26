/*
 * cmd_version.h
 *
 *  Created on: Nov 30, 2018
 *      Author: kevin
 */

#include "cmd.h"

#ifndef CMD_CMD_VERSION_H_
#define CMD_CMD_VERSION_H_

int cmd_version_write_cb(uint8_t op_code, struct cmd_data *pdata, void *arg);
int cmd_version_read_cb(uint8_t op_code, struct cmd_data *pdata, void *arg);
int cmd_version_init(void);

#endif /* CMD_CMD_VERSION_H_ */
