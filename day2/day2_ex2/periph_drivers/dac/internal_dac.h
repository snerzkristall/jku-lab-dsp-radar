// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for internal ADC */

#ifndef INT_DAC_H
#define INT_DAC_H

#include <stdint.h>
#include "tim_hal.h"

//extern uint16_t int_dac1_dma_buffer[], int_dac2_dma_buffer[];
#define INT_DAC_MAX_BUFFER_LENGTH (4096)

struct int_dac_dev_s
{    
    DAC_HandleTypeDef *hdac;
    struct tim_dev_s *tim_dev;
    int8_t (*set_nsamp) (struct int_dac_dev_s *self, uint16_t nsamp);
    int8_t (*set_sample) (struct int_dac_dev_s *self, uint16_t val, uint16_t idx);
    int8_t (*fill_buf) (struct int_dac_dev_s *self, uint16_t *data);
    int8_t (*arm) (struct int_dac_dev_s *self);
    int16_t nsamp;
    //TODO: consider using a vtable
};

void int_dac_dev_init(struct int_dac_dev_s *self, struct tim_dev_s *tim_dev, DAC_HandleTypeDef *hdac);

#endif