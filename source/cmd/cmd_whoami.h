/*
 * cmd_whoami.h
 *
 *  Created on: Nov 25, 2018
 *      Author: kevin
 */

#include "cmd.h"

#ifndef CMD_WHO_AM_I_H_
#define CMD_WHO_AM_I_H_

int cmd_whoami_write_cb(uint8_t op_code, struct cmd_data *pdata, void *arg);
int cmd_whoami_read_cb(uint8_t op_code, struct cmd_data *pdata, void *arg);
int cmd_whoami_init(void);

#endif /* CMD_WHO_AM_I_H_ */
