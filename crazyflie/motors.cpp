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

// Arduino library
#include <oneshot125.hpp>

// Hackflight
#include <firmware/tasks/task1.hpp>

#if defined(BOLT)

static const std::vector<uint8_t> MOTOR_PINS = {PA1, PB11, PA15, PB10};

static const std::vector<uint8_t> POWER_SWITCH_PINS = {PA0, PB12, PC8, PC15};

static auto motors = OneShot125(MOTOR_PINS);

static void enableMotor(const uint8_t id)
{
    pinMode(POWER_SWITCH_PINS[id], OUTPUT);
    digitalWrite(POWER_SWITCH_PINS[id], HIGH);
}

void hf::Task1::motors_init()
{
    enableMotor(0);
    enableMotor(1);
    enableMotor(2);
    enableMotor(3);

    motors.arm();
}


void hf::Task1::motors_setSpeed(uint32_t id, float speed)
{
    const uint8_t pulse_width = 125 * (speed + 1);
    motors.set(id, pulse_width);
}

void hf::Task1::motors_run()
{
    motors.run();
}

#elif defined(NOMOTORS)

void hf::Task1::motors_init()
{
}

void hf::Task1::motors_setSpeed(uint32_t id, float speed)
{
    (void)id;
    (void)speed;
}

void hf::Task1::motors_run()
{
}


#else // cf2

static const uint8_t M1_PIN = PA1;
static const uint8_t M2_PIN = PB11;
static const uint8_t M3_PIN = PA15;
static const uint8_t M4_PIN = PB9;

static uint8_t pulse_widths[4];

void hf::Task1::motors_init()
{
}

void hf::Task1::motors_setSpeed(uint32_t id, float speed)
{
    pulse_widths[id] = (uint8_t)(255 * speed);
}

void hf::Task1::motors_run()
{
    analogWrite(M1_PIN, pulse_widths[0]);
    analogWrite(M2_PIN, pulse_widths[1]);
    analogWrite(M3_PIN, pulse_widths[2]);
    analogWrite(M4_PIN, pulse_widths[3]);
}

#endif
