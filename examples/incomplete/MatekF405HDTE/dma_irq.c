/*
   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stm32f4xx.h>


void handleDmaIrq(uint8_t id);

/*
void DMA2_Stream1_IRQHandler(void) 
{
    handleDmaIrq(0);
}

void DMA2_Stream2_IRQHandler(void) 
{
    handleDmaIrq(1);
}*/
