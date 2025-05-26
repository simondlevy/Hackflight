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

// External libraries
#include <arduino_freertos.h>
#include <BMI088.h>

// Hackflight
#include <system.h>
#include <tasks/imu.hpp>

static const uint8_t FLOWDECK_CS_PIN = 13;

static const uint8_t GYRO_INTERRUPT_PIN = 4;

static Bmi088Accel _accel(Wire, 0x19);
static Bmi088Gyro _gyro(Wire, 0x69);

static ImuTask * _imuTask;

static volatile bool got_gyro_interrupt;

static void gyro_interrupt_handler()
{
    got_gyro_interrupt = true;
}

static void report_forever(const char * msg)
{
    printf("%s\n", msg);
    delay(500);
}

FLASHMEM __attribute__((noinline)) void setup()
{
    Serial.begin(0);
    delay(2'000);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial1.begin(115200);

    vTaskStartScheduler();
}

void loop()
{
}

void ImuTask::readGyroRaw(Axis3i16* dataOut)
{
    (void)dataOut;
}

void ImuTask::readAccelRaw(Axis3i16 * dataOut)
{
    (void)dataOut;
}

bool uartReadByte(uint8_t * byte)
{
    const bool avail = Serial1.available() > 0;

    if (avail) {
        *byte = Serial1.read();
    }

    return avail;
}

void uartWriteByte(const uint8_t byte)
{
    Serial1.write(byte);
}
 
void ImuTask::deviceInit(void)
{
    _imuTask = this;

    int status = _accel.begin();

    if (status < 0) {
        report_forever("Accel Initialization Error");
    }

    _accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_280HZ);
    _accel.setRange(Bmi088Accel::RANGE_24G);

    status = _gyro.begin();

    if (status < 0) {
        report_forever("Gyro Initialization Error");
    }

    _gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);
    _gyro.setRange(Bmi088Gyro::RANGE_2000DPS);
    _gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
    _gyro.mapDrdyInt3(true);

    pinMode(GYRO_INTERRUPT_PIN, arduino::INPUT);

    attachInterrupt(GYRO_INTERRUPT_PIN, gyro_interrupt_handler,
            arduino::RISING);  
}
