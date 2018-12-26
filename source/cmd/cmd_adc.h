/*
 * cmd_adccfg.h
 *
 *  Created on: Dec 25, 2018
 *      Author: kevin
 */

#include "cmd.h"

#ifndef CMD_CMD_ADC_H_
#define CMD_CMD_ADC_H_

int cmd_adchi_write_cb(uint8_t op_code, struct cmd_data *pdata, void *arg);
int cmd_adchi_read_cb(uint8_t op_code, struct cmd_data *pdata, void *arg);
int cmd_adclo_write_cb(uint8_t op_code, struct cmd_data *pdata, void *arg);
int cmd_adclo_read_cb(uint8_t op_code, struct cmd_data *pdata, void *arg);
int cmd_adcstat_write_cb(uint8_t op_code, struct cmd_data *pdata, void *arg);
int cmd_adcstat_read_cb(uint8_t op_code, struct cmd_data *pdata, void *arg);

int cmd_adc_init(void);

#endif /* CMD_CMD_ADC_H_ */
