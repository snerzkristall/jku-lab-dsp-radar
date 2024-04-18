// Copyright 2019-2021, Philipp Peterseil,
// Institute for Communications Engineering and RF-Systems,
// Johannes Kepler University Linz, Austria and all contributors
// SPDX-License-Identifier: MIT

#include "vcp.h"
#include <stdint.h>

#include "usb_device.h"
#include "usbd_cdc_if.h"

// DEFINED IN: usbd_cdc_if.c
extern USBD_HandleTypeDef hUsbDeviceFS;
extern struct sRxBufferHdr rxBufferHdr;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern volatile uint8_t isCDCConnectionOpened;


int vcp_read(struct serial_dev_s* s, void* pBuffer, int size)
{
  if (!rxBufferHdr.ReadDone)
    return 0;

  int remaining = rxBufferHdr.Size - rxBufferHdr.Position;
  int todo = MIN(remaining, size);
  if (todo <= 0)
    return 0;

  memcpy(pBuffer, UserRxBufferFS + rxBufferHdr.Position, todo);
  rxBufferHdr.Position += todo;
  if (rxBufferHdr.Position >= rxBufferHdr.Size)
  {
    rxBufferHdr.ReadDone = 0;
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  }

  return todo;
}

#ifdef USE_USB_HS
enum { kMaxOutPacketSize = CDC_DATA_HS_OUT_PACKET_SIZE };
#else
enum { kMaxOutPacketSize = CDC_DATA_FS_OUT_PACKET_SIZE };
#endif

int vcp_available_for_write(struct serial_dev_s* s) {
  return isCDCConnectionOpened && (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

int vcp_available(struct serial_dev_s* s) {
    return rxBufferHdr.Size - rxBufferHdr.Position;
}

void vcp_init(struct serial_dev_s* s){
    MX_USB_DEVICE_Init();
}

void vcp_deinit(struct serial_dev_s* s){
    USBD_Stop(&hUsbDeviceFS);
}

int vcp_write(struct serial_dev_s* s, const void* pBuffer, int size)
{
  if (size > kMaxOutPacketSize)
  {
    int offset;
    int done = 0;
    for (offset = 0; offset < size; offset += done)
    {
      int todo = MIN(kMaxOutPacketSize, size - offset);
      done = vcp_write(s, ((char*)pBuffer) + offset, todo);
      if (done != todo)
        return offset + done;
    }

    return size;
  }

  USBD_CDC_HandleTypeDef* pCDC =
    (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  while (pCDC->TxState) {} //Wait for previous transfer

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)pBuffer, size);
  if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK)
    return 0;

  uint32_t time = HAL_GetTick();
  while (pCDC->TxState) {
    if (HAL_GetTick() - time > 500) {
      USBD_Stop(&hUsbDeviceFS);
      vcp_init(NULL);
      HAL_Delay(500);
    }
  } //Wait until transfer is done
  return size;
}

void vcp_flush(struct serial_dev_s* s){}
int vcp_setup(struct serial_dev_s* s, int baud, int data, int parity, int stop){ return 0; }

void vcp_serial_dev_init(struct serial_dev_s* s){
    serial_dev_default_init(s);
    s->init = vcp_init;
    s->deinit = vcp_deinit;
    s->read = vcp_read;
    s->write = vcp_write;
    s->available = vcp_available;
    s->available_for_write = vcp_available_for_write;
    s->flush = vcp_flush;
    s->setup = vcp_setup;
}
