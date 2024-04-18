// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for pressure sensor LPS22HH */

#ifndef LPS22HH_H
#define LPS22HH_H

#include <stdint.h>
#include "i2c_hal.h"

/* Register addresses */
#define LPS22HH_WHO_AM_I (0x0F)


struct lps22hh_dev_s
{
    struct i2c_dev_s *i2c_dev; /**< I2C device */
    //struct sai_dev_s *sai_dev; /**< SAI device */
    uint8_t hw_adr; /**< hardware address of chip */
    //uint16_t reg[16]; /**<  */

    // TODO: Add function pointers
};

int8_t lps22hh_writeReg(struct lps22hh_dev_s *self, uint8_t adr, uint16_t val);
int8_t lps22hh_readReg(struct lps22hh_dev_s *self, uint8_t adr, uint8_t *val);

int8_t lps22hh_whoami(struct lps22hh_dev_s *self);
int8_t lps22hh_init(struct lps22hh_dev_s *self);

#endif