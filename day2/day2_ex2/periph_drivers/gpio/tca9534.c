// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for I/O expander TCA9534 */
#include "tca9534.h"
#include <errno.h>
#include <math.h>

int8_t tca9534_writeReg(struct tca9534_dev_s *self, uint8_t adr, uint8_t val)
{
    int8_t result;
    result = self->i2c_dev->i2c_mem_write(self->i2c_dev, self->hw_adr, adr, 1, &val, 1, 0xFFFFFFFF);    
    return result;
}

int8_t tca9534_readReg(struct tca9534_dev_s *self, uint8_t adr, uint8_t *val)
{
    int8_t result;
    result = self->i2c_dev->i2c_mem_read(self->i2c_dev, self->hw_adr, adr, 1, val, 1, 0xFFFFFFFF);    
    return result;
}

// int8_t i2c_mem_read(struct i2c_dev_s *self, uint16_t DevAddress,
//                             uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)

int8_t tca9534_set_port(struct tca9534_dev_s *self, uint8_t val)
{
    int8_t error = tca9534_writeReg(self, TCA9534_OUTPORT_REG_ADR, val);
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

int8_t tca9534_get_port(struct tca9534_dev_s *self, uint8_t *val)
{
    int8_t error = tca9534_readReg(self, TCA9534_INPORT_REG_ADR, val);
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

int8_t tca9534_set_output(struct tca9534_dev_s *self, uint8_t mask)
{
    int8_t error = tca9534_writeReg(self, TCA9534_CONF_REG_ADR, ~mask);
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

int8_t tca9534_init(struct tca9534_dev_s *self)
{
    int8_t error = tca9534_writeReg(self, TCA9534_POLINV_REG_ADR, 0); //disable polarity inversion
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