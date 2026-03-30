/*
   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <Adafruit_VL53L1X.h>

#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/zranger/device_api.h>

static Adafruit_VL53L1X _vl53l1x;

static volatile bool _vl53l1x_is_data_ready;

static void zranger_handle_data_ready()
{
    _vl53l1x_is_data_ready = true;

    _vl53l1x.clearInterrupt();
}

void hf::ZRanger::begin(const uint8_t interruptPin)
{
    Wire1.begin();
    Wire1.setClock(400000);

    if (!_vl53l1x.begin(0x29, &Wire1)) {
        hf::Debugger::reportForever("Unable to initialize sensor");
    }

    if (!_vl53l1x.startRanging()) {
        hf::Debugger::reportForever("Unable to start ranging");
    }

    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
    _vl53l1x.setTimingBudget(50);

    // Polarity=1 => RISING
    _vl53l1x.VL53L1X_SetInterruptPolarity(1);
    attachInterrupt(digitalPinToInterrupt(interruptPin),
            zranger_handle_data_ready, RISING);

    // Clear interrupt to get things started
    _vl53l1x.clearInterrupt();
}

int16_t hf::ZRanger::read()
{
    static int16_t distance;

    if (_vl53l1x_is_data_ready) {
        distance = _vl53l1x.distance();
        _vl53l1x_is_data_ready = false;
    }

    return distance;
}
