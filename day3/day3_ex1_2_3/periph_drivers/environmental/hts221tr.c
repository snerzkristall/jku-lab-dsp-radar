// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for relative humidity and temperature sensor HTS221TR */
#include "hts221tr.h"
#include <errno.h>

int8_t ht221tr_writeReg(struct ht221tr_dev_s *self, uint8_t regadr, uint16_t val)
{
    // uint8_t data[2];
    // data[0]=((regadr&0x7F)<<1)+((val&0x0100)>>8); //left 7 bits are register adr, one data bit (MSB) right
    // data[1]=val&0x00FF; //8 lower data bits
    // return self->i2c_dev->i2c_master_transmit(self->i2c_dev, self->hw_adr, &data[0], 2, 0xFFFFFFFF);
    return 0;
}

int8_t ht221tr_readReg(struct ht221tr_dev_s *self, uint8_t adr, uint8_t *val)
{
    return self->i2c_dev->mem_read(self->i2c_dev, self->hw_adr, adr, 1, val, 1, 0xFFFFFFFF);
}

int8_t ht221tr_whoami(struct ht221tr_dev_s *self)
{
    int8_t error=0;
    uint8_t val=0;
    error+=ht221tr_readReg(self, HTS221TR_WHO_AM_I, &val);
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

int8_t ht221tr_init(struct ht221tr_dev_s *self)
{
    int8_t error=0;
    error+=ht221tr_whoami(self);
    return error;
}