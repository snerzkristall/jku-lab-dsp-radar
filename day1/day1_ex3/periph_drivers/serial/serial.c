// Copyright 2019-2021, Philipp Peterseil,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

#include "serial.h"
#include <string.h>

int serial_print(struct serial_dev_s* s, const char* str){
    return s->write(s, (char*)str, strlen(str));
}

int serial_println(struct serial_dev_s* s, const char* str){
    int len = 0;
    len += s->print(s, str);
    len += s->print(s, EOL);
    return len;
}

void serial_dev_default_init(struct serial_dev_s* s){
    s->print = serial_print;
    s->println = serial_println;
}