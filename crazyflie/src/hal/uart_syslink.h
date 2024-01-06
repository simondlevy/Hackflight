#pragma once

#include <tasks/syslink.hpp>

void uartslkEnableIncoming();
void uartslkGetPacketBlocking(syslinkPacket_t* slp);
void uartslkInit(void);
void uartslkSendDataDmaBlocking(uint32_t size, uint8_t* data);
void uartSyslinkDumpDebugProbe();

