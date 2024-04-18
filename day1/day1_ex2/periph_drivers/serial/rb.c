// Copyright 2019-2021, Philipp Peterseil,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

#include "rb.h"

struct rb_handle_s rb_init(char* mem, uint16_t size){
	struct rb_handle_s h;
	h.mem = mem;
	h.size = size;
	h.back = 0;
	h.front = 0;
	h.count = 0;
	return h;
}


bool rb_put(struct rb_handle_s* h, char e){
	if(h->count < h->size){
		h->mem[h->front++] = e;
		h->front = h->front % h->size;
		h->count++;
		return true;
	}
	else{
		return false;
	}
}

bool rb_get(struct rb_handle_s* h, char* e){
	if(rb_peek(h, e)){
		h->back = ++h->back % h->size;
		h->count--;
		return true;
	}
	return false;
}
bool rb_peek(struct rb_handle_s* h, char* e){
	if(h->count){
		if(e){
			*e = h->mem[h->back];
		}
		return true;
	}
	return false;
		
}

uint8_t rb_count(struct rb_handle_s* h){
	return h->count;
}