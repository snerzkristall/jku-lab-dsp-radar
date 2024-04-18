// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for internal ADC */

#ifndef INT_ADC_H
#define INT_ADC_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define INT_ADC_MAX_BUFFER_LENGTH (4096)

struct int_adc_dev_s
{    
    ADC_HandleTypeDef *hadc;
    struct tim_dev_s *tim_dev;
    int8_t (*set_nsamp) (struct int_adc_dev_s *self, uint16_t nsamp);
    int8_t (*arm) (struct int_adc_dev_s *self);  //arms ADC
    uint32_t* (*get_data) (struct int_adc_dev_s *self); //returns pointer to sampled data
    uint8_t data_avail; //flag showing availability of data (conversion done)
    int16_t nsamp;
    //TODO: consider using a vtable
};

void int_adc_dev_init(struct int_adc_dev_s *self, struct tim_dev_s *tim_dev, ADC_HandleTypeDef *hadc);

#endif