#pragma once

#include <stdint.h>

void spi2_send_byte(uint8_t byte);

void spi2_dma_read(uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

void spi2_begin(void);

