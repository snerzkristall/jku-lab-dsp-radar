// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for I2C based on HAL functions */

#include <errno.h>
#include "stm32h7xx_hal.h"
#include "i2c_hal.h"

void i2c_init(struct i2c_dev_s *self, I2C_HandleTypeDef *hi2c)
{
    self->hi2c = hi2c;
    self->mem_read = &i2c_mem_read;
    self->mem_write = &i2c_mem_write;
    self->master_transmit = &i2c_master_transmit;
}

int8_t i2c_mem_read(struct i2c_dev_s *self, uint16_t DevAddress,
                            uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{    
    if (HAL_I2C_Mem_Read(self->hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)==HAL_OK)
    {
        return 0;
    }
    else
    {
        errno = EIO; //TODO: use/translate error returned from HAL
        return -1;
    }
}


int8_t i2c_mem_write(struct i2c_dev_s *self, uint16_t DevAddress,
                            uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{    
    if (HAL_I2C_Mem_Write(self->hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)==HAL_OK)
    {
        return 0;
    }
    else
    {
        errno = EIO; //TODO: use/translate error returned from HAL
        return -1;
    }
}

int8_t i2c_master_transmit(struct i2c_dev_s *self, uint16_t DevAddress,
                            uint8_t *pData, uint16_t Size, uint32_t Timeout)
{    
    if (HAL_I2C_Master_Transmit(self->hi2c, DevAddress, pData, Size, Timeout)==HAL_OK)
    {
        return 0;
    }
    else
    {
        errno = EIO; //TODO: use/translate error returned from HAL
        return -1;
    }
}