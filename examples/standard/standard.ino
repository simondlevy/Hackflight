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

    pinMode(GYRO_INTERRUPT_PIN, arduino::INPUT);
    attachInterrupt(GYRO_INTERRUPT_PIN, gyro_interrupt_handler,
            arduino::RISING);  
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
    (void)byte;
    return false;
}

void uartWriteByte(const uint8_t byte)
{
    (void)byte;
}
 
void ImuTask::deviceInit(void)
{
    _imuTask = this;
}


