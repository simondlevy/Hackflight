/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 */

#pragma once

#include <string.h>

#include <free_rtos.h>
#include <task.h>

#include <Arduino.h>

#include <tasks/power.hpp>

#include <config.h>
#include <ledseq.h>
#include <platform_defaults.h>
#include <system.h>
#include <worker.hpp>

void pmSyslinkUpdate(syslinkPacket_t *slp);

typedef void (*graceful_shutdown_callback_t)();

class PowerMonitorTask {

    private:
    
        typedef enum
        {
            battery,
            charging,
            charged,
            lowPower,
            shutDown,
        } PMStates;

    public:

        typedef struct {

            union {

                uint8_t flags;
                struct
                {
                    uint8_t isCharging   : 1;
                    uint8_t usbPluggedIn : 1;
                    uint8_t canCharge    : 1;
                    uint8_t unused       : 5;
                };
            };
            float vBat;
            float chargeCurrent;

        }  __attribute__((packed)) syslinkInfo_t;

        // Shared with logger
        float    batteryVoltage;
        uint16_t batteryVoltageMV;
        float    extBatteryVoltage;
        uint16_t extBatteryVoltageMV;
        float    extBatteryCurrent;
        uint8_t  batteryLevel;
        PMStates state;

        // Shared with params
        float batteryCriticalLowVoltage = DEFAULT_BAT_CRITICAL_LOW_VOLTAGE;
        float batteryLowVoltage = DEFAULT_BAT_LOW_VOLTAGE;

        void begin(syslinkInfo_t & pmSyslinkInfo, Worker * worker)
        {
            if(_didInit) {
                return;
            }

            _worker = worker;

            xTaskCreateStatic(
                    pmTask, 
                    "POWER", 
                    PM_TASK_STACKSIZE, 
                    NULL, 
                    0, 
                    taskStackBuffer,
                    &taskTaskBuffer);

            _didInit = true;

            pmSyslinkInfo.vBat = 3.7f;
            setBatteryVoltage(pmSyslinkInfo.vBat); //TODO remove

            _batteryVoltageMin = BATTERY_VOLTAGE_MIN_INIT;
            _ignoreChargedState = false;
        }

        float getBatteryVoltage(void)
        {
            return batteryVoltage;
        }

        void syslinkUpdate(syslinkPacket_t *slp)
        {
            extern syslinkInfo_t pmSyslinkInfo;

            if (slp->type == SYSLINK_PM_BATTERY_STATE) {
                // First byte of the packet contains some PM flags such as USB
                // power, charging etc.
                memcpy(&pmSyslinkInfo, &slp->data[0], sizeof(pmSyslinkInfo));

                // If using voltage measurements from external battery, we'll set the
                // voltage to this instead of the one sent from syslink.
                if (_isExtBatVoltDeckPinSet) {
                    setBatteryVoltage(extBatteryVoltage);
                } else {
                    setBatteryVoltage(pmSyslinkInfo.vBat);
                }

            } else if (slp->type == SYSLINK_PM_SHUTDOWN_REQUEST) {
                extern Worker worker;
                worker.schedule(gracefulShutdown, NULL);
            }
        }

        bool test(void)
        {
            return _didInit;
        }

    private:

        static constexpr float BATTERY_VOLTAGE_MIN_INIT = 6.0;

        static const size_t  PM_TASK_STACKSIZE  = configMINIMAL_STACK_SIZE;

        static const auto PM_BAT_LOW_TIMEOUT = 
            M2T(1000 * DEFAULT_BAT_LOW_DURATION_TO_TRIGGER_SEC);

        static const uint8_t GRACEFUL_SHUTDOWN_MAX_CALLBACKS = 5;

        static constexpr float VREF = 3.0;

        static constexpr float LIPO_TYPICAL_CHARGE_CURVE[10] = {

            3.00, // 00%
            3.78, // 10%
            3.83, // 20%
            3.87, // 30%
            3.89, // 40%
            3.92, // 50%
            3.96, // 60%
            4.00, // 70%
            4.04, // 80%
            4.10  // 90%
        };

        static void pmTask(void *param)
        {
            ((PowerMonitorTask *)param)->run();
        }

        static void gracefulShutdown(void * params)
        {
            ((PowerMonitorTask *)params)->gracefulShutdown();
        }

        bool      _didInit;
        float     _batteryVoltageMin;
        float     _batteryVoltageMax;
        uint8_t   _extBatVoltDeckPin;
        bool      _isExtBatVoltDeckPinSet;
        float     _extBatVoltMultipler;
        uint8_t   _extBatCurrDeckPin;
        bool      _isExtBatCurrDeckPinSet;
        float     _extBatCurrAmpPerVolt;
        uint32_t  _batteryLowTimeStamp;
        uint32_t  _batteryCriticalLowTimeStamp;
        bool      _ignoreChargedState;
        int       _gracefulShutdownCallbacksIndex;

        graceful_shutdown_callback_t 
            graceful_shutdown_callbacks[GRACEFUL_SHUTDOWN_MAX_CALLBACKS];

        StackType_t  taskStackBuffer[PM_TASK_STACKSIZE]; 
        StaticTask_t taskTaskBuffer;

        Worker * _worker;

        void run(void)
        {
            PMStates stateOld = battery;
            uint32_t tickCount;

            vTaskSetApplicationTaskTag(0, (TaskHookFunction_t)TASK_PM_ID_NBR);

            tickCount = xTaskGetTickCount();
            _batteryLowTimeStamp = tickCount;
            _batteryCriticalLowTimeStamp = tickCount;

            systemWaitStart();

            // Continuous battery voltage and status messages must be enabled
            // after system startup to avoid syslink queue overflow.
            enableBatteryStatusAutoupdate();

            while (true) {

                vTaskDelay(100);
                tickCount = xTaskGetTickCount();

                extBatteryVoltage = measureExtBatteryVoltage();
                extBatteryVoltageMV = (uint16_t)(extBatteryVoltage * 1000);
                extBatteryCurrent = pmMeasureExtBatteryCurrent();
                batteryLevel = getBatteryChargeFromVoltage(getBatteryVoltage()) * 10;

                if (getBatteryVoltage() > batteryLowVoltage) {
                    _batteryLowTimeStamp = tickCount;
                }
                if (getBatteryVoltage() > batteryCriticalLowVoltage) {
                    _batteryCriticalLowTimeStamp = tickCount;
                }

                state = updateState();

                if (state != stateOld) {

                    // Actions on state change
                    switch (state) {
                        case charged:
                            //ledseqShowCharged();
                            break;
                        case charging:
                            //ledseqShowCharging();
                            break;
                        case lowPower:
                            //ledseqShowLowPower();
                            break;
                        case battery:
                            //ledseqShowBattery();
                            break;
                        default:
                            break;
                    }
                    stateOld = state;
                }
                // Actions during state
                switch (state) {
                    case charged:
                        break;
                    case charging:
                        {
                            // Charge level between 0.0 and 1.0
                            float chargeLevel = getBatteryChargeFromVoltage(
                                    getBatteryVoltage()) / 10.0f;
                            ledseqSetChargeLevel(chargeLevel);
                        }
                        break;
                    case lowPower:
                        {
                            // auto batteryCriticalLowTime = 
                            //    tickCount - _batteryCriticalLowTimeStamp;
                        }
                        break;
                    case battery:
                        break;
                    default:
                        break;
                }
            }
        }

        void enableBatteryStatusAutoupdate()
        {
            syslinkPacket_t slp = {
                .type = SYSLINK_PM_BATTERY_AUTOUPDATE,
            };

            syslinkSendPacket(&slp);
        }

        float analogReadVoltage(const uint8_t pin)
        {
            return analogRead(pin) * VREF / 4096;
        }

        float measureExtBatteryVoltage(void)
        {
            return _isExtBatVoltDeckPinSet ?
                analogReadVoltage(_extBatVoltDeckPin) * _extBatVoltMultipler : 
                0;
        }

        float pmMeasureExtBatteryCurrent(void)
        {
            return _isExtBatCurrDeckPinSet ? 
                analogReadVoltage(_extBatCurrDeckPin) * _extBatCurrAmpPerVolt : 
                0;
        }

        /**
         * Returns a number from 0 to 9 where 0 is completely discharged
         * and 9 is 90% charged.
         */
        int32_t getBatteryChargeFromVoltage(float voltage)
        {
            return voltage < LIPO_TYPICAL_CHARGE_CURVE[0] ? 0 :
                voltage > LIPO_TYPICAL_CHARGE_CURVE[9] ? 9 :
                voltage >  LIPO_TYPICAL_CHARGE_CURVE[0] ? 1 :
                0;
        }

        PMStates updateState()
        {
            extern syslinkInfo_t pmSyslinkInfo;

            auto usbPluggedIn = pmSyslinkInfo.usbPluggedIn;

            auto isCharging = pmSyslinkInfo.isCharging;

            auto batteryLowTime = xTaskGetTickCount() - _batteryLowTimeStamp;

            auto nextState = 
                _ignoreChargedState ? battery :
                usbPluggedIn && !isCharging ? charged :
                usbPluggedIn && isCharging ? charging :
                battery;

            return nextState == battery && batteryLowTime > PM_BAT_LOW_TIMEOUT ? 
                lowPower :
                nextState;
        }


        /**
         * Sets the battery voltage and its min and max values
         */
        void setBatteryVoltage(float voltage)
        {
            batteryVoltage = voltage;
            batteryVoltageMV = (uint16_t)(voltage * 1000);
            if (_batteryVoltageMax < voltage) {
                _batteryVoltageMax = voltage;
            }
            if (_batteryVoltageMin > voltage) {
                _batteryVoltageMin = voltage;
            }
        }

        void gracefulShutdown(void)
        {
            for (int i = 0; i < _gracefulShutdownCallbacksIndex; i++) {
                graceful_shutdown_callback_t callback = graceful_shutdown_callbacks[i];

                callback();
            }

            syslinkPacket_t slp = {
                .type = SYSLINK_PM_SHUTDOWN_ACK,
            };

            syslinkSendPacket(&slp);
        }
};
