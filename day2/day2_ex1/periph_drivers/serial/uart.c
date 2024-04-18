// Copyright 2019-2021, Philipp Peterseil,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT


#include "uart.h"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    struct serial_dev_s* inst = (struct serial_dev_s*)(huart->pRxBuffPtr);
    if(rb_get((struct rb_handle_s*)inst->txbuffer, (char*)&inst->temporary_buffer)){
        HAL_UART_Transmit_IT((UART_HandleTypeDef*)inst->handle, (uint8_t*)&inst->temporary_buffer, 1);
    }
    else{
        ((UART_HandleTypeDef*)inst->handle)->pTxBuffPtr = (uint8_t*)((UART_HandleTypeDef*)inst->handle)->pRxBuffPtr;
        HAL_UART_Receive_IT((UART_HandleTypeDef*)inst->handle, (uint8_t*)&inst->temporary_buffer, 1);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    struct serial_dev_s* inst = (struct serial_dev_s*)(huart->pTxBuffPtr);
    rb_put((struct rb_handle_s*)inst->rxbuffer, (char)inst->temporary_buffer);
    HAL_UART_Receive_IT((UART_HandleTypeDef*)inst->handle, (uint8_t*)&inst->temporary_buffer, 1);
}


int uart_read(struct serial_dev_s* s, void* pBuffer, int size)
{
    uint16_t count = rb_count((struct rb_handle_s*)s->rxbuffer);

    uint16_t i;
    for(i = 0; i < size && i < count; i++){
        rb_get((struct rb_handle_s*)s->rxbuffer, &((char*)pBuffer)[i]);
    }
    
    return i;
}



int uart_available_for_write(struct serial_dev_s* s) {
  return 1; // hw uart is always ready
}

int uart_available(struct serial_dev_s* s) {
    return rb_count((struct rb_handle_s*)s->rxbuffer); //number to read
}

void uart_init(struct serial_dev_s* s){    
    HAL_UART_Init((UART_HandleTypeDef*)s->handle);

    ((UART_HandleTypeDef*)s->handle)->pTxBuffPtr = (uint8_t*)s;
    HAL_UART_Receive_IT((UART_HandleTypeDef*)s->handle, (uint8_t*)&s->temporary_buffer, 1);
    
}

void uart_deinit(struct serial_dev_s* s){
    HAL_UART_Abort((UART_HandleTypeDef*)s->handle);
    HAL_UART_DeInit((UART_HandleTypeDef*)s->handle);
}

int uart_write(struct serial_dev_s* s, const void* pBuffer, int size)
{
    uint16_t i;
    for(i = 0; i < size; i++){
        if(!rb_put((struct rb_handle_s*)s->txbuffer, ((char*)pBuffer)[i])) break;
    }

    UART_HandleTypeDef* huart = (UART_HandleTypeDef*)s->handle;
    if(huart->gState != HAL_UART_STATE_BUSY_TX){
        HAL_UART_AbortReceive_IT(huart);
        while(huart->gState != HAL_UART_STATE_READY){} // TIMEOUT !!!

        rb_get((struct rb_handle_s*)s->txbuffer, (char*)&s->temporary_buffer);

        ((UART_HandleTypeDef*)s->handle)->pRxBuffPtr = (uint8_t*)s;
        HAL_UART_Transmit_IT((UART_HandleTypeDef*)s->handle, (uint8_t*)&s->temporary_buffer, 1);
    }


    return i;
}

void uart_flush(struct serial_dev_s* s){
    while(rb_count((struct rb_handle_s*)s->txbuffer)){}
}

int uart_setup(struct serial_dev_s* s, int baud, int data, int parity, int stop){ 
    UART_HandleTypeDef* huart = (UART_HandleTypeDef*)s->handle;

    if(huart->gState != HAL_UART_STATE_READY) return false;

    huart->Init.BaudRate = baud;
    huart->Init.WordLength = /*(bits == 9) ? UART_WORDLENGTH_9B : */ UART_WORDLENGTH_8B;
    huart->Init.StopBits = (stop==2) ? UART_STOPBITS_2 : UART_STOPBITS_1;
    huart->Init.Parity = (parity == 0) ? UART_PARITY_NONE : ((parity == 1) ? UART_PARITY_ODD : UART_PARITY_EVEN) ;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    return true; 
}


void uart_serial_dev_init(struct serial_dev_s* s, UART_HandleTypeDef* h, struct rb_handle_s* rxbuffer, struct rb_handle_s* txbuffer){
    serial_dev_default_init(s);
    s->handle = (void*)h;
    s->rxbuffer = (void*)rxbuffer;
    s->txbuffer = (void*)txbuffer;
    s->init = uart_init;
    s->deinit = uart_deinit;
    s->read = uart_read;
    s->write = uart_write;
    s->available = uart_available;
    s->available_for_write = uart_available_for_write;
    s->flush = uart_flush;
    s->setup = uart_setup;
}

