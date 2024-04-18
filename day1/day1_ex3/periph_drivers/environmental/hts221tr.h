// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for relative humidity and temperature sensor HTS221TR */

#ifndef HTS221TR_H
#define HTS221TR_H

#include <stdint.h>
#include "i2c_hal.h"

/* Register addresses */
#define HTS221TR_WHO_AM_I (0x0F)
#define HTS221TR_AV_CONF (0x10)
#define HTS221TR_CTRL_REG1 (0x20)
#define HTS221TR_CTRL_REG2 (0x21)
#define HTS221TR_CTRL_REG3 (0x22)
#define HTS221TR_STATUS_REG (0x27)
#define HTS221TR_HUMIDITY_OUT_L (0x28)
#define HTS221TR_HUMIDITY_OUT_H (0x29)
#define HTS221TR_TEMP_OUT_L (0x2A)
#define HTS221TR_TEMP_OUT_H (0x2B)

/* AV_CONF entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define HTS221TR_AVGT2_BIT_NUM (5u)
#define HTS221TR_AVGT1_BIT_NUM (4u)
#define HTS221TR_AVGT0_BIT_NUM (3u)
#define HTS221TR_AVGH2_BIT_NUM (2u)
#define HTS221TR_AVGH1_BIT_NUM (1u)
#define HTS221TR_AVGH0_BIT_NUM (0u)
#define HTS221TR_AVGT_BIT_MASK (0b00111000)
#define HTS221TR_AVGH_BIT_MASK (0b00000111)


struct ht221tr_dev_s
{
    struct i2c_dev_s *i2c_dev; /**< I2C device */
    //struct sai_dev_s *sai_dev; /**< SAI device */
    uint8_t hw_adr; /**< hardware address of chip */
    //uint16_t reg[16]; /**<  */

    // TODO: Add function pointers
};

int8_t ht221tr_writeReg(struct ht221tr_dev_s *self, uint8_t adr, uint16_t val);
int8_t ht221tr_readReg(struct ht221tr_dev_s *self, uint8_t adr, uint8_t *val);

int8_t ht221tr_whoami(struct ht221tr_dev_s *self);
int8_t ht221tr_init(struct ht221tr_dev_s *self);

#endif