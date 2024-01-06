/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 *
 * Platform functionality for the CF2 platform
 */

#define DEBUG_MODULE "PLATFORM"

#include <string.h>

#include <hal/exti.h>
#include <hal/nvic.h>

#include "platform.h"

const platformConfig_t* platformGetListOfConfigurations(int* nrOfConfigs) {
  
    extern const MotorPerifDef* motorMapDefaultBrushed[NBR_OF_MOTORS];

    *nrOfConfigs = 1;

    static platformConfig_t config = {};

    strcpy(config.deviceType, "CF21");
    strcpy(config.deviceTypeName, "Crazyflie 2.1");
    config.sensorImplementation = SensorImplementation_bmi088_bmp388;
    config.physicalLayoutAntennasAreClose = true;
    config.motorMap = motorMapDefaultBrushed;
  
    return &config;
}

void platformInitCrazyflie() {
  //Low level init: Clock and Interrupt controller
  nvicInit();

  //EXTI interrupts
  extiInit();
}


// Config functions ------------------------

const char* platformConfigGetPlatformName() {
  return "cf2";
}
