// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for I2C based on HAL functions */
#ifndef SPI_HAL_H
#define SPI_HAL_H

#include "stm32h7xx_hal.h"

struct spi_dev_s
{    
    SPI_HandleTypeDef *hspi;
    int8_t (*transmit) (struct spi_dev_s *self,
                                   uint8_t *pData, uint16_t Size, uint32_t Timeout);
    int8_t (*transmitReceive) (struct spi_dev_s *self, uint8_t *pTxData,
                                    uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
    //TODO: consider using a vtable
};

int8_t spi_transmit(struct spi_dev_s *self,
                            uint8_t *pData, uint16_t Size, uint32_t Timeout);

int8_t spi_transmitReceive(struct spi_dev_s *self, uint8_t *pTxData,
                            uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

#endif