// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for I/O expander TCA9534 */
#ifndef TCA9534_H
#define TCA9534_H

#include <stdint.h>
#include "i2c_hal.h"

/* Register addresses */
#define TCA9534_INPORT_REG_ADR (0x0)
#define TCA9534_OUTPORT_REG_ADR (0x1)
#define TCA9534_POLINV_REG_ADR (0x2)
#define TCA9534_CONF_REG_ADR (0x3)

struct tca9534_dev_s
{
    struct i2c_dev_s *i2c_dev; /**< I2C device */
    uint8_t hw_adr; /**<  */
    //uint8_t reg0; /**<  */

    // TODO: Add function pointers
};

int8_t tca9534_writeReg(struct tca9534_dev_s *self, uint8_t adr, uint8_t val);
//int8_t tca9534_readReg(struct tca9534_dev_s *self, uint8_t adr, uint8_t val);

int8_t tca9534_init(struct tca9534_dev_s *self);
int8_t tca9534_set_port(struct tca9534_dev_s *self, uint8_t val);
int8_t tca9534_set_output(struct tca9534_dev_s *self, uint8_t mask);
int8_t tca9534_get_port(struct tca9534_dev_s *self, uint8_t *val);

#endif