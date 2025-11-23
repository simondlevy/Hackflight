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

#include <__messages__.h>
#include <serializer.hpp>
#include <zranger.hpp>

static const char * BTNAME = "Goku"; 

static const uint8_t RX1_PIN = 4;
static const uint8_t TX1_PIN = 14;

static const uint8_t RX2_PIN = 15;
static const uint8_t TX2_PIN = 27;

static VL53L1X vl53l1x;

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

void sensor_task(void *) 
{
    /*
    ZRanger::init(&Wire, vl53l1x);

    PMW3901 pmw3901;

    SPI.begin();

    if (!pmw3901.begin()) {
        error("PMW3901");
    }
    */

    MspSerializer serializer = {};

    HardwareSerial uart(2);
    uart_begin(uart, RX2_PIN, TX2_PIN);

    while (true) {

        /*
        const float zrange = vl53l1x.read();

        int16_t deltaX = 0;
        int16_t deltaY = 0;
        bool gotMotion = false;

        pmw3901.readMotion(deltaX, deltaY, gotMotion);

        const int16_t msg[4] = {
            (int16_t)zrange,
            deltaX,
            deltaY,
            (int16_t)gotMotion
        };

        */

        const int16_t msg[4] = {1, 2, 3, 4};

        serializer.serializeShorts(MSP_SENSORS, msg, 4);

        for (uint8_t k=0; k<serializer.payloadSize; ++k) {
            uart.write(serializer.payload[k]);
        }

        vTaskDelay(10);
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

    static Task task4 = Task(sensor_task, 3);
    task4.run();
}

void loop()
{
}
