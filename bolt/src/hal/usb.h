#pragma once

static const uint8_t USB_RX_TX_PACKET_SIZE = 64;

/* Structure used for in/out data via USB */
typedef struct
{
  uint8_t size;
  uint8_t data[USB_RX_TX_PACKET_SIZE];
} USBPacket;


#ifdef __cplusplus
extern "C" {
#endif

    void usbInit(void);

#ifdef __cplusplus
}
#endif
