/*
 * i2c_cfg.h
 *
 *  Created on: Nov 30, 2018
 *      Author: kevin
 */

#ifndef I2C_CFG_H_
#define I2C_CFG_H_

/** The base addres for the I2C peripheral block to use in slave mode. */
#define I2C_CFG_SLAVE_BASE (I2C0_BASE)

/** The clock frequency for the I2C peripheral block. */
#define I2C_CFG_CLOCK_FREQUENCY (12000000)

/** The 7-bit I2C bus address for the slave device. */
#define I2C_CFG_ADDR_7BIT (0x7EU)
#if I2C_CFG_ADDR_7BIT >= 0x7F
#error "I2C_CFG_ADDR_7BIT must but <= 0x7F"
#endif

/** The maximum number of bytes allowed on this device (MAX is 255!). */
#define I2C_CFG_MAX_DATA_LENGTH (34)
#if I2C_CFG_MAX_DATA_LENGTH > 255
#error "I2C_CFG_MAX_DATA_LENGTH must be <= 255"
#endif

#endif /* I2C_CFG_H_ */
