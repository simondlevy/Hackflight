/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * Initialize the SPI.
 */
void spiBeginTransaction(uint16_t baudRatePrescaler);
/* Send the data_tx buffer and receive into the data_rx buffer */
bool spiExchange(size_t length, const uint8_t *data);

typedef enum {

    MSBFIRST,
    LSBFIRST,

} spi_endianness_t;

typedef enum {

    SPI_MODE0,
    SPI_MODE1,
    SPI_MODE2,
    SPI_MODE3,

} spi_mode_t;

class SPISettings {

    friend class SPIClass;

    public:

    SPISettings(
            const uint32_t rate, 
            const spi_endianness_t endianness, 
            const spi_mode_t mode)
    {
        _rate = rate;
        _endianness = endianness;
        _mode = mode;
    }

    private:

    uint32_t _rate; 
    spi_endianness_t _endianness; 
    spi_mode_t _mode;

};

class SPIClass {

    public:

        void begin(void);

        void beginTransaction(const SPISettings & settings);

        void transfer(uint8_t * data, size_t size);

        void endTransaction(void);
};
