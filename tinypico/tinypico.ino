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
#include <TinyPICO.h>

static const char * BTNAME = "Hackflight-TinyPICO"; 

static const uint8_t TXD1 = 14;
static const uint8_t RXD1 = 4;

static BluetoothSerial bts; 

static const uint32_t TIMEOUT_MSEC = 1000;

static HardwareSerial uarts(1);

static TaskHandle_t bt_to_uart_task_handle = NULL;

static TinyPICO tp = TinyPICO();

static bool connected;

void bt_to_uart_task(void *) 
{
    static uint32_t last_received;

    while (true) {

        while (bts.available()) {
            last_received = millis();
            connected = true;
            const uint8_t b = bts.read();
            uarts.write(b);
        }

        if (millis() - last_received > TIMEOUT_MSEC) {
            connected = false;
        }

        vTaskDelay(1);
    }
}

static TaskHandle_t uart_to_bt_task_handle = NULL;

void uart_to_bt_task(void *) 
{
    while (true) {

        while (uarts.available()) {
            const uint8_t b = uarts.read();
            bts.write(b);
        }

        vTaskDelay(1);
    }
}

static TaskHandle_t blink_task_handle = NULL;

void blink_task(void *) 
{
    while (true) {

        tp.DotStar_SetPixelColor(0, 64, 0);

        vTaskDelay(500);

        if (!connected) {
            tp.DotStar_SetPixelColor(0, 0, 0);
            vTaskDelay(500);
        }
    }
}

void setup() 
{
    uarts.begin(115200, SERIAL_8N1, RXD1, TXD1);

    bts.begin(BTNAME);

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

    xTaskCreate(
            blink_task, 
            "blink_task", 
            10000,             // Stack size (bytes)
            NULL,        
            2,                 // Priority
            &blink_task_handle);}

void loop()
{
}
