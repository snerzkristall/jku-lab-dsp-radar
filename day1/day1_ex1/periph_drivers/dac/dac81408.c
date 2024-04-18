// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for DAC DAC81408 */
#include "dac81408.h"
#include <errno.h>

#define DMA_BUFFER \
    __attribute__((section(".dma_buffer"))) __attribute__ ((aligned (4)))

DMA_BUFFER uint32_t dac81408_dma_buffer[DAC81408_BUFFER_LENGTH]; //configure datawidth for DMA as word (=32 bit), because of 24 bit transfers


int8_t dac81408_writeReg(struct dac81408_dev_s *self, uint8_t adr, uint16_t val)
{
    uint32_t data=((adr&(0b111111))<<16) | (val);
    return self->spi_dev->spi_transmit(self->spi_dev, (uint8_t*) &data, 1, 0xFFFFFFFF);      
}

int8_t dac81408_readReg(struct dac81408_dev_s *self, uint8_t adr, uint16_t *retval)
{   
    int8_t error;
    uint32_t dataRx;    
    uint32_t dataTx=(1<<23)|((adr&(0b111111))<<16)|(0);

    self->spi_dev->spi_transmit(self->spi_dev, (uint8_t*) &dataTx, 1, 0xFFFFFFFF);    
    error=self->spi_dev->spi_transmitReceive(self->spi_dev, (uint8_t*) &dataTx, &dataRx, 1, 0xFFFFFFFF);
    *retval=(uint16_t)((dataRx)&(0xFFFF));
    uint8_t adr_rx=(int8_t)(((dataRx&(~(1<<23)))&(0xFF0000))>>16);
    if(adr_rx!=adr)
    {
        errno = EIO;
        error=-1;
    }
    return error;
}

int8_t dac81408_init(struct dac81408_dev_s *self)
{
    uint16_t tmp;
    int8_t error=0;
    error+=dac81408_writeReg(self, DAC81408_SPICONFIG, 0b0101010100100 & ~(1<<5)); //disable power down
    error+=dac81408_readReg(self, DAC81408_DEVICEID, &tmp);
    if((tmp>>2) != 0x298)
    {
        //wrong device ID
        error=-1;
        return error;
    }
    error+=dac81408_set_range(self, bipo_10);
    error+=dac81408_writeReg(self, DAC81408_DACPWDWN, 0xF00F); //disable power down    
    error+=dac81408_writeReg(self, DAC81408_GENCONFIG, 0b0011111100000000); //enable internal reference
    error+=dac81408_writeReg(self, DAC81408_SYNCCONFIG, 0x0); //asynchronous mode
    error+=dac81408_writeReg(self, DAC81408_BRDCONFIG, 0b1111000000001111); //ignore BRDCAST commands
    return error;
}

int8_t dac81408_set_range(struct dac81408_dev_s *self, enum dac81408_range range)
{
    uint16_t val;
    int8_t error=0;
    switch(range)
    {
        case bipo_10:
            val=0xAAAA;
        break;
        default:
            error=-1;
            return error;
        break;
    }
    error+=dac81408_writeReg(self, DAC81408_DACRANGE0, val);    
    error+=dac81408_writeReg(self, DAC81408_DACRANGE1, val);
    return error;
}
