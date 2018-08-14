/*
 * This file is part of Hackflight.
 *
 * Hackflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Hackflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3

#define USABLE_TIMER_CHANNEL_COUNT 17

#define USE_UART1
#define USE_UART2
#define USE_UART3

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF (BIT(0)|BIT(1))
