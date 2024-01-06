#pragma once

#include <tasks/syslink.hpp>

#define USB_RX_TX_PACKET_SIZE   (64)

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
    bool usbGetDataBlocking(USBPacket *in);
    bool usbSendData(uint32_t size, uint8_t* data);

#ifdef __cplusplus
}
#endif
