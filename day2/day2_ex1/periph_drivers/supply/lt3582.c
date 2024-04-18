// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for programmable DC/DC converter LT3582 */
#include "lt3582.h"
#include <errno.h>
#include <math.h>

void lt3582_init(struct lt3582_dev_s *self, struct i2c_dev_s *i2c_dev, uint8_t hw_adr)
{
    self->i2c_dev = i2c_dev;
    self->setVoltages = &lt3582_setVoltages;
    self->hw_adr = hw_adr;
}

int8_t lt3582_writeReg(struct lt3582_dev_s *self, uint8_t adr, uint8_t val)
{
    int8_t result;
    result = self->i2c_dev->mem_write(self->i2c_dev, self->hw_adr, adr, 1, &val, 1, 0xFFFFFFFF);    
    return result;
}

int8_t lt3582_setVoltages(struct lt3582_dev_s *self, float_t volt_p, float_t volt_n)
{
    int8_t error=0;
    uint8_t reg0val=(uint8_t)((volt_p-3.2f)/50e-3);
    uint8_t vplusbit=0;

    if (fabs(volt_p-(3.2f+reg0val*50e-3)) < fabs(volt_p-(3.2f+reg0val*50e-3+25e-3)))
        vplusbit=0;
    else
        vplusbit=1;
    
    self->reg0=reg0val;//136;
    error += lt3582_writeReg(self, LT3582_REG0_ADR, self->reg0);
        
    self->reg1=(uint8_t)(-(volt_n+1.2)/50e-3);//176;
    error += lt3582_writeReg(self, LT3582_REG1_ADR, self->reg1);

    self->reg2 = (vplusbit<<LT3582_VPLUS_BIT_NUM) | (1<<LT3582_PDDIS_BIT_NUM) | LT3582_PUSEQ_MASK;
    error += lt3582_writeReg(self, LT3582_REG2_ADR, self->reg2);

    self->cmdr = (1<<LT3582_RSEL2_BIT_NUM) | (1<<LT3582_RSEL1_BIT_NUM) | (1<<LT3582_RSEL0_BIT_NUM);
    error += lt3582_writeReg(self, LT3582_CMDR_ADR, self->cmdr);
    
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
