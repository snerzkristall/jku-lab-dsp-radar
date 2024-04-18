// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for audio codec WM8731 */
#include "wm8731.h"
#include <errno.h>
#include <memory.h>

static volatile uint8_t wm8731_nextOutBuf=0, //next available half of output buffer (0 or 1)
                        wm8731_outBufAvail=0, //0 while waiting for next interrupt
                        wm8731_nextInBuf=0, //next available half of input buffer (0 or 1)
                        wm8731_inBufAvail=0; //0 while waiting for next interrupt

static struct wm8731_dev_s *wm8731_devGlob=0; //required for ISR/Callback fct.

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    wm8731_nextOutBuf=0;
    wm8731_outBufAvail=1;
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    wm8731_nextInBuf=0;
    wm8731_inBufAvail=1;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
    wm8731_nextOutBuf=1;
    wm8731_outBufAvail=1;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
    wm8731_nextInBuf=1;
    wm8731_inBufAvail=1;
}

void wm8731_init(struct wm8731_dev_s *self, struct i2c_dev_s *i2c_dev,
                 SAI_HandleTypeDef *sai_dev_dac, SAI_HandleTypeDef *sai_dev_adc, uint8_t hw_adr)
{
    self->hw_adr = hw_adr;
    self->sai_dev_adc = sai_dev_adc;
    self->sai_dev_dac = sai_dev_dac;
    self->i2c_dev = i2c_dev;
    self->reset = &wm8731_reset;
    self->disable_power_down = &wm8731_disable_power_down;
    self->set_interface_format = &wm8731_set_interface_format;
    self->set_sampling_rate = &wm8731_set_sampling_rate;
    self->conf_linein = &wm8731_conf_linein;
    self->activate = &wm8731_activate;
    self->setup = &wm8731_setup;

    self->waitOutBuf = &wm8731_waitOutBuf;
    self->waitInBuf = &wm8731_waitInBuf;
    self->startDacDma = &wm8731_startDacDma;
    self->startAdcDma = &wm8731_startAdcDma;
    self->putOutBuf = &wm8731_putOutBuf;
    self->getInBuf = &wm8731_getInBuf;
}


void wm8731_waitOutBuf(struct wm8731_dev_s *self)
{
    while(!wm8731_outBufAvail)
    {
    }    
}

void wm8731_waitInBuf(struct wm8731_dev_s *self)
{
    while(!wm8731_inBufAvail)
    {
    }    
}

void wm8731_startDacDma(struct wm8731_dev_s *self)
{  
  if (HAL_SAI_Transmit_DMA(self->sai_dev_dac, (uint8_t*)wm8731_dacBuf, WM8731_DAC_BUF_LEN) != HAL_OK)
  {
    Error_Handler();
  }
}

void wm8731_startAdcDma(struct wm8731_dev_s *self)
{  
  if (HAL_SAI_Receive_DMA(self->sai_dev_adc, (uint8_t*)wm8731_adcBuf, WM8731_ADC_BUF_LEN) != HAL_OK)
  {
    Error_Handler();
  }
}

void wm8731_putOutBuf(struct wm8731_dev_s *self, int16_t *data)
{
    wm8731_outBufAvail=0;
    uint16_t offset=wm8731_nextOutBuf*(WM8731_ADC_BUF_LEN/2);
    void *adr_dest;
    adr_dest=(void*) (&wm8731_dacBuf[offset]);
    memcpy(adr_dest, data, WM8731_DAC_BUF_LEN);
    __DSB(); //wait for end of data transfer
}

void wm8731_getInBuf(struct wm8731_dev_s *self, int16_t *data)
{
    wm8731_inBufAvail=0;
    uint16_t offset=wm8731_nextInBuf*(WM8731_ADC_BUF_LEN/2);
    void *adr_src;
    adr_src=(void*) (&wm8731_adcBuf[offset]);
    memcpy(data, adr_src, WM8731_ADC_BUF_LEN);
    __DSB(); //wait for end of data transfer
}

int8_t wm8731_writeReg(struct wm8731_dev_s *self, uint8_t regadr, uint16_t val)
{
    uint8_t data[2];
    data[0]=((regadr&0x7F)<<1)+((val&0x0100)>>8); //left 7 bits are register adr, one data bit (MSB) right
    data[1]=val&0x00FF; //8 lower data bits
    return self->i2c_dev->master_transmit(self->i2c_dev, self->hw_adr, &data[0], 2, 0xFFFFFFFF);
    //HAL_I2C_Master_Transmit(&hi2c2, addr, &data[0], size, timeout);
}

int8_t wm8731_reset(struct wm8731_dev_s *self)
{
    int8_t error=0;
    self->reg[WM8731_RESET_ADR]=0u; //writing 0 resets device
    error+=wm8731_writeReg(self, WM8731_RESET_ADR, self->reg[WM8731_RESET_ADR]);
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

int8_t wm8731_disable_power_down(struct wm8731_dev_s *self)
{
    int8_t error=0;
    self->reg[WM8731_PWR_DOWN_CTRL_ADR]=0u; //writing 0 enables all components
    error+=wm8731_writeReg(self, WM8731_PWR_DOWN_CTRL_ADR, self->reg[WM8731_PWR_DOWN_CTRL_ADR]);
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

int8_t wm8731_set_interface_format(struct wm8731_dev_s *self)
{    
    int8_t error=0;
    //16bit, DSP Mode: MSB on 1st BCLK, WM8731 is master
    self->reg[WM8731_DIG_INTERFACE_FMT_ADR] = 0u;
    self->reg[WM8731_DIG_INTERFACE_FMT_ADR] &= ~(1<<WM8731_BCLKINV_BIT_NUM); //0: clk not inverted
    // self->reg[WM8731_DIG_INTERFACE_FMT_ADR] |= (1<<WM8731_BCLKINV_BIT_NUM); //1: clk inverted
    self->reg[WM8731_DIG_INTERFACE_FMT_ADR] |= (1<<WM8731_MS_BIT_NUM); //1: WM8731 is master
    self->reg[WM8731_DIG_INTERFACE_FMT_ADR] &= ~(1<<WM8731_LRSWAP_BIT_NUM); //0: L/R not swapped
    self->reg[WM8731_DIG_INTERFACE_FMT_ADR] &= ~(1<<WM8731_LRP_BIT_NUM); //0: MSB is available on 1st BCLK rising edge after DACLRC rising edge
    // self->reg[WM8731_DIG_INTERFACE_FMT_ADR] |= (1<<WM8731_LRP_BIT_NUM); //1: MSB is available on 2nd BCLK rising edge after DACLRC rising edge
    self->reg[WM8731_DIG_INTERFACE_FMT_ADR] &= ~(WM8731_IWL_MASK); //all 0: 16 bits
    self->reg[WM8731_DIG_INTERFACE_FMT_ADR] |= (WM8731_FORMAT_MASK); //all 1: DSP Mode, frame sync + 2 data packed word
    error+=wm8731_writeReg(self, WM8731_DIG_INTERFACE_FMT_ADR, self->reg[WM8731_DIG_INTERFACE_FMT_ADR]);

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

int8_t wm8731_set_sampling_rate(struct wm8731_dev_s *self, enum wm8731_sr sr)
{
    int8_t error=0;
    switch(sr)
    {
        case ADC48_DAC48:
            self->reg[WM8731_SAMPLING_CTRL_ADR]=0u;
            self->reg[WM8731_SAMPLING_CTRL_ADR] &= ~(1<<WM8731_CLKODIV2_BIT_NUM); //0: clk out not divided
            self->reg[WM8731_SAMPLING_CTRL_ADR] &= ~(1<<WM8731_CLKIDIV2_BIT_NUM); //0: clk in not divided
            self->reg[WM8731_SAMPLING_CTRL_ADR] &= ~(WM8731_SR_MASK);
            self->reg[WM8731_SAMPLING_CTRL_ADR] &= ~(1<<WM8731_BOSR_BIT_NUM);
            self->reg[WM8731_SAMPLING_CTRL_ADR] |= (1<<WM8731_USB_NORM_BIT_NUM); //1: USB mode (clk is 12 MHz)
            break;
        case ADC8_DAC8:
            self->reg[WM8731_SAMPLING_CTRL_ADR]=0u;
            self->reg[WM8731_SAMPLING_CTRL_ADR] &= ~(1<<WM8731_CLKODIV2_BIT_NUM); //0: clk out not divided
            self->reg[WM8731_SAMPLING_CTRL_ADR] &= ~(1<<WM8731_CLKIDIV2_BIT_NUM); //0: clk in not divided
            self->reg[WM8731_SAMPLING_CTRL_ADR] |= (3<<WM8731_SR_BIT_NUM);
            self->reg[WM8731_SAMPLING_CTRL_ADR] &= ~(1<<WM8731_BOSR_BIT_NUM);
            self->reg[WM8731_SAMPLING_CTRL_ADR] |= (1<<WM8731_USB_NORM_BIT_NUM); //1: USB mode (clk is 12 MHz)
            break;
        default:
            errno=EINVAL;
            return -1;
            break;
    }
    error+=wm8731_writeReg(self, WM8731_SAMPLING_CTRL_ADR, self->reg[WM8731_SAMPLING_CTRL_ADR]);

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

int8_t wm8731_conf_analog_path(struct wm8731_dev_s *self)
{
    int8_t error=0;
    self->reg[WM8731_ANALOG_AUDIO_PATH_CTRL_ADR]=0u;
    self->reg[WM8731_ANALOG_AUDIO_PATH_CTRL_ADR] &= ~(WM8731_SIDEATT_MASK); //all 0: 6dB attenuation of sidetone
    self->reg[WM8731_ANALOG_AUDIO_PATH_CTRL_ADR] &= ~(1<<WM8731_SIDETONE_BIT_NUM); //0: no sidetone
    self->reg[WM8731_ANALOG_AUDIO_PATH_CTRL_ADR] |= (1<<WM8731_DACSEL_BIT_NUM); //1: enable DAC
    self->reg[WM8731_ANALOG_AUDIO_PATH_CTRL_ADR] &= ~(1<<WM8731_BYPASS_BIT_NUM); //0: no bypass
    self->reg[WM8731_ANALOG_AUDIO_PATH_CTRL_ADR] &= ~(1<<WM8731_INSEL_BIT_NUM); //0: select line in
    self->reg[WM8731_ANALOG_AUDIO_PATH_CTRL_ADR] &= ~(1<<WM8731_MUTEMIC_BIT_NUM); //0: disable micmute
    self->reg[WM8731_ANALOG_AUDIO_PATH_CTRL_ADR] &= ~(1<<WM8731_MICBOOST_BIT_NUM); //0: no mic boost

    error+=wm8731_writeReg(self, WM8731_ANALOG_AUDIO_PATH_CTRL_ADR, self->reg[WM8731_ANALOG_AUDIO_PATH_CTRL_ADR]);

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

int8_t wm8731_conf_digital_path(struct wm8731_dev_s *self)
{
    int8_t error=0;
    self->reg[WM8731_DIG_AUDIO_PATH_CTRL_ADR]=0u;
    self->reg[WM8731_DIG_AUDIO_PATH_CTRL_ADR] &= ~(1<<WM8731_HPOR_BIT_NUM); //0: clear dc offset when highpass enabled
    self->reg[WM8731_DIG_AUDIO_PATH_CTRL_ADR] &= ~(1<<WM8731_DACMU_BIT_NUM); //0: disable DAC mute
    self->reg[WM8731_DIG_AUDIO_PATH_CTRL_ADR] &= ~(WM8731_DEEMPH_MASK); //00: disable de-emphasis control
    self->reg[WM8731_DIG_AUDIO_PATH_CTRL_ADR] &= ~(1<<WM8731_ADCHPD_BIT_NUM); //0: enable ADC highpass filter

    error+=wm8731_writeReg(self, WM8731_DIG_AUDIO_PATH_CTRL_ADR, self->reg[WM8731_DIG_AUDIO_PATH_CTRL_ADR]);

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

int8_t wm8731_conf_linein(struct wm8731_dev_s *self, float_t volume_db)
{
    int8_t error=0;
    self->reg[WM8731_LEFT_LINE_IN_ADR]=0u;
    self->reg[WM8731_LEFT_LINE_IN_ADR] &= ~(1<<WM8731_LRINBOTH_BIT_NUM); //0: decouple left and right channel
    self->reg[WM8731_LEFT_LINE_IN_ADR] &= ~(1<<WM8731_LINMUTE_BIT_NUM); //0: disable mute
    self->reg[WM8731_LEFT_LINE_IN_ADR] |= (0b10111<<WM8731_LINVOL_BIT_NUM)&WM8731_LINVOL_MASK; //0b10111: default 0dB
    error+=wm8731_writeReg(self, WM8731_LEFT_LINE_IN_ADR, self->reg[WM8731_LEFT_LINE_IN_ADR]);

    self->reg[WM8731_RIGHT_LINE_IN_ADR]=0u;
    self->reg[WM8731_RIGHT_LINE_IN_ADR] &= ~(1<<WM8731_RLINBOTH_BIT_NUM); //0: decouple left and right channel
    self->reg[WM8731_RIGHT_LINE_IN_ADR] &= ~(1<<WM8731_RINMUTE_BIT_NUM); //0: disable mute
    self->reg[WM8731_RIGHT_LINE_IN_ADR] |= (0b10111<<WM8731_RINVOL_BIT_NUM)&WM8731_RINVOL_MASK; //0b10111: default 0dB
    error+=wm8731_writeReg(self, WM8731_RIGHT_LINE_IN_ADR, self->reg[WM8731_RIGHT_LINE_IN_ADR]);

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

int8_t wm8731_activate(struct wm8731_dev_s *self)
{
    int8_t error=0;
    self->reg[WM8731_ACTIVE_CTRL_ADR]=0u|(1<<WM8731_ACTIVE_BIT_NUM);
    error+=wm8731_writeReg(self, WM8731_ACTIVE_CTRL_ADR, self->reg[WM8731_ACTIVE_CTRL_ADR]);
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

int8_t wm8731_setup(struct wm8731_dev_s *self, enum wm8731_sr sr)
{
    int8_t error=0;
    error+=wm8731_reset(self);
    error+=wm8731_disable_power_down(self);
    error+=wm8731_set_interface_format(self);
    error+=wm8731_set_sampling_rate(self, sr);
    error+=wm8731_conf_analog_path(self);
    error+=wm8731_conf_linein(self, 0);
    error+=wm8731_conf_digital_path(self);
    error+=wm8731_activate(self);
    wm8731_devGlob=self;
    return error;
}