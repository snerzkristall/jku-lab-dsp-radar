// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for pressure sensor LPS22HH */
#include "lps22hh.h"
#include <errno.h>

int8_t lps22hh_writeReg(struct lps22hh_dev_s *self, uint8_t regadr, uint16_t val)
{
    // uint8_t data[2];
    // data[0]=((regadr&0x7F)<<1)+((val&0x0100)>>8); //left 7 bits are register adr, one data bit (MSB) right
    // data[1]=val&0x00FF; //8 lower data bits
    // return self->i2c_dev->i2c_master_transmit(self->i2c_dev, self->hw_adr, &data[0], 2, 0xFFFFFFFF);
    return 0;
}

int8_t lps22hh_readReg(struct lps22hh_dev_s *self, uint8_t adr, uint8_t *val)
{
    return self->i2c_dev->mem_read(self->i2c_dev, self->hw_adr, adr, 1, val, 1, 0xFFFFFFFF);
}

int8_t lps22hh_whoami(struct lps22hh_dev_s *self)
{
    int8_t error=0;
    uint8_t val=0;
    error+=lps22hh_readReg(self, LPS22HH_WHO_AM_I, &val);
    // val should be 0b10110011
    if(error)
    {
        errno=EIO;
        return -1;
    }
    else
    {
        errno=0;
        return 0;
    }
}

int8_t lps22hh_init(struct lps22hh_dev_s *self)
{
    int8_t error=0;
    error+=lps22hh_whoami(self);
    return error;
}