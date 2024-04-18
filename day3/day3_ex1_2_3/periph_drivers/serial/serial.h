// Copyright 2019-2021, Philipp Peterseil,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

#ifndef __SERIAL_DRV_H__
#define __SERIAL_DRV_H__

#include <stdint.h>

#define EOL "\r\n"

struct serial_dev_s {
    uint16_t temporary_buffer;
    void* rxbuffer;
    void* txbuffer;
    void* handle;
    //// implement basic methods for specific hardware
    void (*init)(struct serial_dev_s* s);
    void (*deinit)(struct serial_dev_s* s);
    int (*read)(struct serial_dev_s* s, void* pBuffer, int size);
    int (*write)(struct serial_dev_s* s, const void* pBuffer, int size);
    int (*available)(struct serial_dev_s* s);
    int (*available_for_write)(struct serial_dev_s* s);
    void (*flush)(struct serial_dev_s* s);
    int (*setup)(struct serial_dev_s* s, int baud, int data, int parity, int stop);

    //// implement higher level methods below
    int (*print)(struct serial_dev_s* s, const char* str);
    int (*println)(struct serial_dev_s* s, const char* str);
};

void serial_dev_default_init(struct serial_dev_s* s);

#endif