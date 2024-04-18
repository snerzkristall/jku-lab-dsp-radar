// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for DAC DAC81408 */

#ifndef DAC81408_H
#define DAC81408_H

#include <stdint.h>
#include "spi_hal.h"

/* Register addresses */
#define DAC81408_NOP          (0x00)    //NOP Register
#define DAC81408_DEVICEID     (0x01)    //Device ID Register
#define DAC81408_STATUS       (0x02)    //Status Register
#define DAC81408_SPICONFIG    (0x03)    //SPI Configuration Register
#define DAC81408_GENCONFIG    (0x04)    //General Configuration Register
#define DAC81408_BRDCONFIG    (0x05)    //Broadcast Configuration Register
#define DAC81408_SYNCCONFIG   (0x06)    //Sync Configuration Register
#define DAC81408_TOGGCONFIG0  (0x07)    //DAC[7:4] Toggle Configuration Register
#define DAC81408_TOGGCONFIG1  (0x08)    //DAC[3:0] Toggle Configuration Register
#define DAC81408_DACPWDWN     (0x09)    //DAC Power-Down Register
#define DAC81408_DACRANGE0    (0x0B)    //DAC[7:4] Range Register
#define DAC81408_DACRANGE1    (0x0C)    //DAC[3:0] Range Register
#define DAC81408_TRIGGER      (0x0E)    //Trigger Register
#define DAC81408_BRDCAST      (0x0F)    //Broadcast Data Register
#define DAC81408_DAC0         (0x14)    //DAC0 Data Register
#define DAC81408_DAC1         (0x15)    //DAC1 Data Register
#define DAC81408_DAC2         (0x16)    //DAC2 Data Register
#define DAC81408_DAC3         (0x17)    //DAC3 Data Register
#define DAC81408_DAC4         (0x18)    //DAC4 Data Register
#define DAC81408_DAC5         (0x19)    //DAC5 Data Register
#define DAC81408_DAC6         (0x1A)    //DAC6 Data Register
#define DAC81408_DAC7         (0x1B)    //DAC7 Data Register
#define DAC81408_OFFSET0      (0x21)    //DAC[6-7;4-5] Differential Offset Register
#define DAC81408_OFFSET1      (0x22)    //DAC[2-3;0-1] Differential Offset Register

struct dac81408_dev_s
{
    struct spi_dev_s *spi_dev; /**< SPI device */
    // TODO: Add function pointers
};

enum dac81408_range
{
unip_5, //0 to 5 V
bipo_10 //-10 to 10 V
};

int8_t dac81408_writeReg(struct dac81408_dev_s *self, uint8_t adr, uint16_t val);
int8_t dac81408_readReg(struct dac81408_dev_s *self, uint8_t adr, uint16_t *retval);
int8_t dac81408_init(struct dac81408_dev_s *self);
int8_t dac81408_set_range(struct dac81408_dev_s *self, enum dac81408_range range);

#define DAC81408_BUFFER_LENGTH (128)
extern uint32_t dac81408_dma_buffer[];


#endif