// Copyright 2019-2021, Reinhard Feger,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

/* Driver for audio codec WM8731 */

#ifndef WM8731_H
#define WM8731_H

#include <stdint.h>
#include "i2c_hal.h"
#include "main.h"

/* Register addresses */
#define WM8731_LEFT_LINE_IN_ADR (0x0)
#define WM8731_RIGHT_LINE_IN_ADR (0x1)
#define WM8731_LEFT_HEAD_OUT_ADR (0x2)
#define WM8731_RIGHT_HEAD_OUT_ADR (0x3)
#define WM8731_ANALOG_AUDIO_PATH_CTRL_ADR (0x4)
#define WM8731_DIG_AUDIO_PATH_CTRL_ADR (0x5)
#define WM8731_PWR_DOWN_CTRL_ADR (0x6)
#define WM8731_DIG_INTERFACE_FMT_ADR (0x7)
#define WM8731_SAMPLING_CTRL_ADR (0x8)
#define WM8731_ACTIVE_CTRL_ADR (0x9)
#define WM8731_RESET_ADR (0xF)

/* REG0 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define WM8731_LRINBOTH_BIT_NUM (8u)
#define WM8731_LINMUTE_BIT_NUM (7u)
#define WM8731_LINVOL_BIT_NUM (0u)
#define WM8731_LINVOL_MASK (0b000011111)

/* REG1 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define WM8731_RLINBOTH_BIT_NUM (8u)
#define WM8731_RINMUTE_BIT_NUM (7u)
#define WM8731_RINVOL_BIT_NUM (0u)
#define WM8731_RINVOL_MASK (0b000011111)

/* REG2 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define WM8731_LRHPBOTH_BIT_NUM (8u)
#define WM8731_LZCEN_BIT_NUM (7u)
#define WM8731_LHPVOL_BIT_NUM (0u)
#define WM8731_LHPVOL_MASK (0b001111111)

/* REG3 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define WM8731_RLHPBOTH_BIT_NUM (8u)
#define WM8731_RZCEN_BIT_NUM (7u)
#define WM8731_RHPVOL_BIT_NUM (0u)
#define WM8731_RHPVOL_MASK (0b001111111)

/* REG4 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define WM8731_SIDEATT_BIT_NUM (6u)
#define WM8731_SIDEATT_MASK (0b011000000)
#define WM8731_SIDETONE_BIT_NUM (5u)
#define WM8731_DACSEL_BIT_NUM (4u)
#define WM8731_BYPASS_BIT_NUM (3u)
#define WM8731_INSEL_BIT_NUM (2u)
#define WM8731_MUTEMIC_BIT_NUM (1u)
#define WM8731_MICBOOST_BIT_NUM (0u)

/* REG5 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define WM8731_HPOR_BIT_NUM (4u)
#define WM8731_DACMU_BIT_NUM (3u)
#define WM8731_DEEMPH_BIT_NUM (1u)
#define WM8731_DEEMPH_MASK (0b000000110)
#define WM8731_ADCHPD_BIT_NUM (0u)

/* REG6 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define WM8731_POWEROFF_BIT_NUM (7u)
#define WM8731_CLKOUTPD_BIT_NUM (6u)
#define WM8731_OSCPD_BIT_NUM (5u)
#define WM8731_OUTPD_BIT_NUM (4u)
#define WM8731_DACPD_BIT_NUM (3u)
#define WM8731_ADCPD_BIT_NUM (2u)
#define WM8731_MICPD_BIT_NUM (1u)
#define WM8731_LINEINPD_BIT_NUM (0u)

/* REG7 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define WM8731_BCLKINV_BIT_NUM (7u)
#define WM8731_MS_BIT_NUM (6u)
#define WM8731_LRSWAP_BIT_NUM (5u)
#define WM8731_LRP_BIT_NUM (4u)
#define WM8731_IWL_BIT_NUM (2u)
#define WM8731_IWL_MASK (0b000001100)
#define WM8731_FORMAT_BIT_NUM (0u)
#define WM8731_FORMAT_MASK (0b000000011)

/* REG8 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define WM8731_CLKODIV2_BIT_NUM (7u)
#define WM8731_CLKIDIV2_BIT_NUM (6u)
#define WM8731_SR_BIT_NUM (2u)
#define WM8731_SR_MASK (0b000111100)
#define WM8731_BOSR_BIT_NUM (1u)
#define WM8731_USB_NORM_BIT_NUM (0u)

/* REG9 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define WM8731_ACTIVE_BIT_NUM (0u)

/* REG15 entries: BIT_NUM: position of LSB of corresponding value, MASK covers all bits of corresponding value */
#define WM8731_RESET_BIT_NUM (0u)
#define WM8731_RESET_MASK (0b111111111)

#define WM8731_DAC_BUF_LEN 512 //total length (words), half of it is used for double buffering
#define WM8731_ADC_BUF_LEN 512 //total length (words), half of it is used for double buffering
#define DMA_BUFFER \
    __attribute__((section(".dma_buffer"))) __attribute__ ((aligned (4)))
DMA_BUFFER static int16_t wm8731_dacBuf[WM8731_DAC_BUF_LEN];
DMA_BUFFER static int16_t wm8731_adcBuf[WM8731_ADC_BUF_LEN];

enum wm8731_sr {ADC48_DAC48,  ADC8_DAC8};

struct wm8731_dev_s
{
    struct i2c_dev_s *i2c_dev; /**< I2C device */
    SAI_HandleTypeDef *sai_dev_dac;
    SAI_HandleTypeDef *sai_dev_adc;
    int8_t (*reset) (struct wm8731_dev_s *self);
    int8_t (*disable_power_down) (struct wm8731_dev_s *self);
    int8_t (*set_interface_format) (struct wm8731_dev_s *self);
    int8_t (*set_sampling_rate) (struct wm8731_dev_s *self, enum wm8731_sr sr);
    int8_t (*conf_linein) (struct wm8731_dev_s *self, float_t volume_db);
    int8_t (*activate) (struct wm8731_dev_s *self);
    int8_t (*setup) (struct wm8731_dev_s *self, enum wm8731_sr sr);

    void (*waitOutBuf) (struct wm8731_dev_s *self);
    void (*waitInBuf) (struct wm8731_dev_s *self);
    void (*startDacDma) (struct wm8731_dev_s *self);
    void (*startAdcDma) (struct wm8731_dev_s *self);
    void (*putOutBuf) (struct wm8731_dev_s *self, int16_t *data);
    void (*getInBuf) (struct wm8731_dev_s *self, int16_t *data);

    uint8_t hw_adr; /**< hardware address of chip */
    uint16_t reg[16]; /**<  */
};

void wm8731_init(struct wm8731_dev_s *self, struct i2c_dev_s *i2c_dev,
                 SAI_HandleTypeDef *sai_dev_dac, SAI_HandleTypeDef *sai_dev_adc, uint8_t hw_adr);

int8_t wm8731_writeReg(struct wm8731_dev_s *self, uint8_t adr, uint16_t val);
//int8_t wm8731_readReg(struct lt3582_dev_s *self, uint8_t adr, uint8_t &val);

int8_t wm8731_reset(struct wm8731_dev_s *self);
int8_t wm8731_disable_power_down(struct wm8731_dev_s *self);
int8_t wm8731_set_interface_format(struct wm8731_dev_s *self);
int8_t wm8731_set_sampling_rate(struct wm8731_dev_s *self, enum wm8731_sr sr);
int8_t wm8731_conf_linein(struct wm8731_dev_s *self, float_t volume_db);
int8_t wm8731_activate(struct wm8731_dev_s *self);
int8_t wm8731_setup(struct wm8731_dev_s *self, enum wm8731_sr sr);

void wm8731_waitOutBuf(struct wm8731_dev_s *self);
void wm8731_waitInBuf(struct wm8731_dev_s *self);
void wm8731_startDacDma(struct wm8731_dev_s *self);
void wm8731_startAdcDma(struct wm8731_dev_s *self);
void wm8731_putOutBuf(struct wm8731_dev_s *self, int16_t *data);
void wm8731_getInBuf(struct wm8731_dev_s *self, int16_t *data);

#endif