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
#include <pmw3901.hpp>
#include <TinyPICO.h>
#include <VL53L1X.h>

static const char * BTNAME = "Goku"; 

static const uint8_t TXD1 = 14;
static const uint8_t RXD1 = 4;

static BluetoothSerial bts; 

static const uint32_t TIMEOUT_MSEC = 1000;

static HardwareSerial uart1(1);

static TinyPICO tp = TinyPICO();

static bool connected;

class Task {

    private:

        typedef void (*fun_t)(void * obj);
        
        TaskHandle_t _handle;
        fun_t _fun;
        uint8_t _priority;

    public:

        Task(const fun_t fun, const uint8_t priority) 
        {
            _fun = fun;
            _priority = priority;
        }

        void run() 
        {
            xTaskCreate(
                    _fun, 
                    "",      // name
                    10000,   // stack size (bytes)
                    NULL,    // data
                    _priority,
                    &_handle);
        }

};

//////////////////////////////////////////////////////////

void bt_to_uart_task(void *) 
{
    static uint32_t last_received;

    while (true) {

        while (bts.available()) {
            last_received = millis();
            connected = true;
            const uint8_t b = bts.read();
            uart1.write(b);
        }

        if (millis() - last_received > TIMEOUT_MSEC) {
            connected = false;
        }

        vTaskDelay(1);
    }
}

//////////////////////////////////////////////////////////

void uart_to_bt_task(void *) 
{
    while (true) {

        while (uart1.available()) {
            const uint8_t b = uart1.read();
            bts.write(b);
        }

        vTaskDelay(1);
    }
}

//////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////

void setup() 
{
    uart1.begin(115200, SERIAL_8N1, RXD1, TXD1);

    bts.begin(BTNAME);

    static Task _task1 = Task(bt_to_uart_task, 1);
    _task1.run();

    static Task _task2 = Task(uart_to_bt_task, 1);
    _task2.run();

    static Task _task3 = Task(blink_task, 2);
    _task3.run();
}

void loop()
{
}
