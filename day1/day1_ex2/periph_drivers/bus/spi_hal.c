// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for I2C based on HAL functions */

#include <errno.h>
#include "stm32h7xx_hal.h"
#include "spi_hal.h"

int8_t spi_transmit(struct spi_dev_s *self,
                            uint8_t *pData, uint16_t Size, uint32_t Timeout)
{   
    if (HAL_SPI_Transmit(self->hspi, pData, Size, Timeout)==HAL_OK)
    {
        return 0;
    }
    else
    {
        errno = EIO; //TODO: use/translate error returned from HAL
        return -1;
    }
}

int8_t spi_transmitReceive(struct spi_dev_s *self, uint8_t *pTxData,
                            uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{   
    if (HAL_SPI_TransmitReceive(self->hspi, pTxData, pRxData, Size, Timeout)==HAL_OK)
    {
        return 0;
    }
    else
    {
        errno = EIO; //TODO: use/translate error returned from HAL
        return -1;
    }
}