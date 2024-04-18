// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for Timer based on HAL functions */

#include <errno.h>
#include "stm32h7xx_hal.h"
#include "tim_hal.h"
#include "errno.h"

int8_t tim_set_prescaler(struct tim_dev_s *self, uint16_t Prescaler);
int8_t tim_set_period(struct tim_dev_s *self, uint16_t Period);
int8_t tim_set_freq(struct tim_dev_s *self, uint32_t Freq);
int8_t tim_start(struct tim_dev_s *self);
int8_t tim_stop(struct tim_dev_s *self);

void tim_dev_init(struct tim_dev_s *self, TIM_HandleTypeDef *htim)
{
    self->htim = htim;
    self->set_period = &tim_set_period;
    self->set_prescaler = &tim_set_prescaler;
    self->set_freq = &tim_set_freq;
    self->start = &tim_start;
    self->stop = &tim_stop;
}

int8_t tim_set_prescaler(struct tim_dev_s *self, uint16_t Prescaler)
{
    //TODO: check for errors/limits
    self->htim->Instance->PSC = Prescaler;
    errno = 0;
    return 0;
}

int8_t tim_set_period(struct tim_dev_s *self, uint16_t Period)
{
    //TODO: check for errors/limits
    self->htim->Instance->ARR = Period; //Autoreload register
    errno = 0;
    return 0;
}

int8_t tim_set_freq(struct tim_dev_s *self, uint32_t Freq)
{
    //TODO: check for errors/limits
    //TODO: read clock configuration    
    //Update Frequency = TIM_CLOCK / ((Prescaler+1)*(Period+1)) 
    //assuming 240 MHz time clock
    self->set_prescaler(self, 6 - 1);  // go down to 40 MHz
    self->set_period(self, ((uint16_t)(40e6/Freq)) - 1);
    errno = 0;
    return 0;
}

int8_t tim_start(struct tim_dev_s *self)
{    
    if(HAL_TIM_Base_Start_IT(self->htim) != HAL_OK)
    {
        /* Start Error */
        //Error_Handler();
        //TODO: errno
        return -1;
    }
    else
    {
        errno = 0;
        return 0;
    }
}

int8_t tim_stop(struct tim_dev_s *self)
{
    if(HAL_TIM_Base_Stop_IT(self->htim) != HAL_OK)
    {
        /* Stop Error */
        //Error_Handler();
        //TODO: errno
        return -1;
    }
    else
    {
        errno = 0;
        return 0;
    }
}