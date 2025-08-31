/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 *
 * nvic.c - Contains all Cortex-M3 processor exceptions handlers
 */


#include <cfassert.h>

#include "exti.h"
#include "usb_dcd_int.h"
#include "usb_core.h"

#define DONT_DISCARD __attribute__((used))

extern void motors_stop(void);

void nvicInit(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

/**
 * @brief  This function handles SysTick Handler.
 */
extern void tickFreeRTOS(void);

void DONT_DISCARD SysTick_Handler(void)
{
    tickFreeRTOS();
}

/**
 * @brief  STM32_USBF_OTG_ISR_Handler
 *         handles all USB Interrupts
 * @param  pdev: device instance
 * @retval status
 */

void  __attribute__((used)) OTG_FS_IRQHandler(void)
{
    extern USB_OTG_CORE_HANDLE USB_OTG_dev;

    USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

/**
 * @brief  This function handles NMI exception.
 */
void DONT_DISCARD NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 */
void DONT_DISCARD HardFault_Handler(void)
{
    //http://www.st.com/mcu/forums-cat-6778-23.html
    //****************************************************
    //To test this application, you can use this snippet anywhere:
    // //Let's crash the MCU!
    // asm (" MOVS r0, #1 \n"
    // " LDM r0,{r1-r2} \n"
    // " BX LR; \n");
    asm( "TST LR, #4 \n"
            "ITE EQ \n"
            "MRSEQ R0, MSP \n"
            "MRSNE R0, PSP \n"
            "B printHardFault");
}

void DONT_DISCARD printHardFault(uint32_t* hardfaultArgs)
{
    while (1) {} 
}

/**
 * @brief  This function handles Memory Manage exception.
 */
void DONT_DISCARD MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    motors_stop();

    while (1)
    {}
}

/**
 * @brief  This function handles Bus Fault exception.
 */
void DONT_DISCARD BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    motors_stop();

    while (1)
    {}
}

/**
 * @brief  This function handles Usage Fault exception.
 */
void DONT_DISCARD UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    motors_stop();

    while (1)
    {}
}

/**
 * @brief  This function handles Debug Monitor exception.
 */
void DONT_DISCARD DebugMon_Handler(void)
{
}
