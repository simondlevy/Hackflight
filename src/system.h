/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
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

void systemInit(const uint8_t led_pin, const uint8_t flowdeck_cs_pin);

const bool systemIsLedInverted();

void systemReportForever(const char * msg);

bool systemUartReadByte(uint8_t *);
            
void systemUartWriteByte(const uint8_t byte);

void systemWaitStart(void);
