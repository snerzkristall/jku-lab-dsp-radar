// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for programmable DC/DC converter LT3582 */
#ifndef LT3582_H
#define LT3582_H

#include <stdint.h>
#include "i2c_hal.h"

/* Register addresses */
#define LT3582_REG0_ADR (0x0)
#define LT3582_REG1_ADR (0x1)
#define LT3582_REG2_ADR (0x2)
#define LT3582_CMDR_ADR (0x4)

/* REG2 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define LT3582_LOCK_BIT_NUM (6u)
#define LT3582_VPLUS_BIT_NUM (5u)
#define LT3582_IRMP_BIT_NUM (3u)
#define LT3582_IRMP_MASK (0b00011000)
#define LT3582_PDDIS_BIT_NUM (2u)
#define LT3582_PUSEQ_BIT_NUM (0u)
#define LT3582_PUSEQ_MASK (0b11)

/* CMDR entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define LT3582_WOTP_BIT_NUM (7u)
#define LT3582_CF_FAULT_BIT_NUM (6u)
#define LT3582_RST_BIT_NUM (5u)
#define LT3582_SWOFF_BIT_NUM (4u)
#define LT3582_RSVD_BIT_NUM (3u)
#define LT3582_RSEL2_BIT_NUM (2u)
#define LT3582_RSEL1_BIT_NUM (1u)
#define LT3582_RSEL0_BIT_NUM (0u)

struct lt3582_dev_s
{
    struct i2c_dev_s *i2c_dev; /**< I2C device */
    int8_t (*setVoltages) (struct lt3582_dev_s *self, float_t volt_p, float_t volt_n);
    uint8_t hw_adr; /**< hardware address of chip */
    uint8_t reg0; /**< VOUTP Output Voltage (00h=3.2V, BFh = 12.75V) */
    uint8_t reg1; /**< VOUTN Output Voltage (00h=1.2V, FFh = 13.95V) */
    uint8_t reg2; /**< Lockout bit, 25mV increase Voutp, RAMP pull-up current, power down discharge EN, power up sequencing */
    uint8_t cmdr; /**< write OTP, clr/progr fault, RST, switches off, REG select (OTP or REG) */
};

int8_t lt3582_writeReg(struct lt3582_dev_s *self, uint8_t adr, uint8_t val);
int8_t lt3582_readReg(struct lt3582_dev_s *self, uint8_t adr, uint8_t val);

int8_t lt3582_setVoltages(struct lt3582_dev_s *self, float_t volt_p, float_t volt_n);
void lt3582_init(struct lt3582_dev_s *self, struct i2c_dev_s *i2c_dev, uint8_t hw_adr);

#endif