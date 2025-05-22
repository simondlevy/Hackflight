#pragma once

#include "bstdr_types.h"

bstdr_ret_t bstdr_burst_read(uint8_t dev_id, uint8_t reg_addr, 
        uint8_t *reg_data, uint32_t len);

bstdr_ret_t bstdr_burst_write(uint8_t dev_id, uint8_t reg_addr, 
        uint8_t *reg_data, uint32_t len);
