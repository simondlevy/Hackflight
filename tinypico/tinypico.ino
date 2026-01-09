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

static const char * BTNAME = "tinypico"; 

static const uint8_t RX1_PIN = 4;
static const uint8_t TX1_PIN = 14;

static BluetoothSerial bts; 

static const uint32_t BT_RX_TIMEOUT_MSEC = 1000;

static TinyPICO tp = TinyPICO();

static bool connected;

static HardwareSerial uart1(1);

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

static void error(const char * sensorName)
{
    while (true) {
        Serial.print("Failed to initialize ");
        Serial.println(sensorName);
        delay(500);
    }
}

static void uart_begin(
        HardwareSerial & uart, const uint8_t rx_pin, const uint8_t tx_pin)
{
    uart.begin(115200, SERIAL_8N1, rx_pin, tx_pin);
}

void bt_to_uart_task(void *) 
{
    static uint32_t last_received;

    uart_begin(uart1, RX1_PIN, TX1_PIN);

    while (true) {

        while (bts.available()) {
            last_received = millis();
            connected = true;
            const uint8_t b = bts.read();
            uart1.write(b);
        }

        if (millis() - last_received > BT_RX_TIMEOUT_MSEC) {
            connected = false;
        }

        vTaskDelay(1);
    }
}

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
    Serial.begin(115200);

    bts.begin(BTNAME);

    static Task task1 = Task(bt_to_uart_task, 1);
    task1.run();

    static Task task2 = Task(uart_to_bt_task, 1);
    task2.run();

    static Task task3 = Task(blink_task, 2);
    task3.run();
}

void loop()
{
}
