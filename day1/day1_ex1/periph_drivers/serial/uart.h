// Copyright 2019-2021, Philipp Peterseil,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

#ifndef __UART_H_
#define __UART_H_

#include <stm32h7xx_hal.h>
#include "serial.h"
#include "rb.h"


void uart_serial_dev_init(struct serial_dev_s* s, UART_HandleTypeDef* h, struct rb_handle_s* rxbuffer, struct rb_handle_s* txbuffer);



#endif