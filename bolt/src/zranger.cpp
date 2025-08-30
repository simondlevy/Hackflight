/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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

#include <hal/i2cdev.h>

#include <tasks/zranger.hpp>
#include <vl53l1.hpp>
#include <st/vl53l1_api.h>

static const uint8_t VL53L1_DEFAULT_ADDRESS = 0x29;

static VL53L1 _vl53l1;

void ZRangerTask::hardware_init()
{
    static const uint8_t VL53L1_DEFAULT_ADDRESS = 0x29;

    _vl53l1.init(&deckBus, VL53L1_DEFAULT_ADDRESS);

    _vl53l1.begin();

    _vl53l1.setDistanceMode(VL53L1::DISTANCE_MODE_MEDIUM);
    _vl53l1.setTimingBudgetMsec(25);
}

float ZRangerTask::hardware_read()
{
    return _vl53l1.readDistance();
}

extern "C" {

    VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t usec)
    {
        void delayMicroseconds(const uint32_t usec);

        delayMicroseconds(usec);

        return VL53L1_ERROR_NONE;
    }

    VL53L1_Error VL53L1_WriteMulti(VL53L1_Dev_t *pdev, uint16_t index, 
            uint8_t * pdata, uint32_t count)
    {
        return i2cdevWriteReg16(
                (I2C_Dev*)pdev->I2Cx, pdev->devAddr, index, count, pdata) ?
            VL53L1_ERROR_NONE : 
            VL53L1_ERROR_CONTROL_INTERFACE;
    }

    VL53L1_Error VL53L1_ReadMulti(VL53L1_Dev_t *pdev, uint16_t index, 
            uint8_t * pdata, uint32_t   count)
    {
        return i2cdevReadReg16(
                (I2C_Dev*)pdev->I2Cx, pdev->devAddr, index, count, pdata) ?
            VL53L1_ERROR_NONE : 
            VL53L1_ERROR_CONTROL_INTERFACE;
    }
}
