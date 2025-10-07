/**
 *
 * Copyright (C) 2025 Simon D. Levy
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

#include <BluetoothSerial.h> 

static BluetoothSerial bts; 

static const uint8_t TXD1 = 14;
static const uint8_t RXD1 = 4;

static HardwareSerial uarts(1);

static TaskHandle_t bt_to_uart_task_handle = NULL;

void bt_to_uart_task(void *parameter) 
{
    while (true) {

        while (bts.available()) {
            const uint8_t b = bts.read();
            uarts.write(b);
        }

        vTaskDelay(1);
    }
}

static TaskHandle_t uart_to_bt_task_handle = NULL;

void uart_to_bt_task(void *parameter) 
{
    while (true) {

        vTaskDelay(1);
    }
}


void setup() 
{
    uarts.begin(115200, SERIAL_8N1, RXD1, TXD1);

    bts.begin("Hackflight"); 

    xTaskCreate(
            bt_to_uart_task, 
            "bt_to_uart_task", 
            10000,             // Stack size (bytes)
            NULL,        
            1,                 // Priority
            &bt_to_uart_task_handle);


    xTaskCreate(
            uart_to_bt_task, 
            "uart_to_bt_task", 
            10000,             // Stack size (bytes)
            NULL,        
            1,                 // Priority
            &uart_to_bt_task_handle);
}

void loop()
{
}


