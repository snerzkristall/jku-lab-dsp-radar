// Copyright 2019-2021, Philipp Peterseil,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

#ifndef __RB_H__
#define __RB_H__

#include <stdbool.h>
#include <stdint.h>

struct rb_handle_s {
	char* mem;
	uint16_t size;
	uint16_t count;
	uint16_t front;
	uint16_t back;
};

struct rb_handle_s rb_init(char* mem, uint16_t size);

bool rb_put(struct rb_handle_s* h, char e);

bool rb_get(struct rb_handle_s* h, char* e);
bool rb_peek(struct rb_handle_s* h, char* e);

uint8_t rb_count(struct rb_handle_s* h);

#endif