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
 */

#include <crossplatform.h>

#include <arduino/digital.h>
#include <arduino/time.h>

#include <led.h>
#include <param_macros.h>

static const uint8_t LED_ENABLE_BITMASK_BIT = 7;

static const uint8_t ERR_LED1      = PIN_LED_RED_L;
static const uint8_t ERR_LED2      = PIN_LED_RED_R;

enum {
    LED_POL_POS,
    LED_POL_NEG
};

typedef enum { 
    LED_LEDSEQ, 
    LED_PARAM_BITMASK 
} ledSwitch_t;

static uint8_t led_polarity[] =
{
    LED_POL_POS, // BLUE_L
    LED_POL_NEG, // GREEN_L
    LED_POL_NEG, // RED_L
    LED_POL_NEG, // GREEN_R
    LED_POL_NEG, // RED_R
};

////////////////////////////////////////////////////////////////////

static bool didInit = 0;
static uint8_t ledControlBitmask;
static uint8_t ledLastState[LED_COUNT];
static ledSwitch_t ledSwitchState;

static void ledSetForce(const uint8_t pin, uint8_t value)
{
    if (led_polarity[pin] == LED_POL_NEG) {
        value = 1 - value;
    }

    digitalWrite(pin, value);
}

static void ledRestoreSavedState(void)
{
    ledSet(PIN_LED_BLUE_L,  ledLastState[0]);
    ledSet(PIN_LED_GREEN_L, ledLastState[1]);
    ledSet(PIN_LED_RED_L,   ledLastState[2]);
    ledSet(PIN_LED_GREEN_R, ledLastState[3]);
    ledSet(PIN_LED_RED_R,   ledLastState[4]);

}

static void ledSetSwitch(ledSwitch_t ledSwitch)
{
    if (ledSwitchState != ledSwitch) {
        ledSwitchState = ledSwitch;
        switch (ledSwitch) {
            case LED_LEDSEQ:
                ledRestoreSavedState();
                break;
            case LED_PARAM_BITMASK:
                break;
            default:
                break;
        }
    }
}

static void ledBitmaskParamCallback(void)
{
    if (ledControlBitmask & (1 << LED_ENABLE_BITMASK_BIT)) {

        ledSetSwitch(LED_PARAM_BITMASK);

        ledSetForce(PIN_LED_BLUE_L,  ledControlBitmask & 0x01);
        ledSetForce(PIN_LED_GREEN_L, ledControlBitmask & 0x02);
        ledSetForce(PIN_LED_RED_L,   ledControlBitmask & 0x04);
        ledSetForce(PIN_LED_GREEN_R, ledControlBitmask & 0x08);
        ledSetForce(PIN_LED_RED_R,   ledControlBitmask & 0x10);
    }
    else {
        ledSetSwitch(LED_LEDSEQ);
    }
}

static void initLed(const uint8_t pin)
{
    pinMode(pin, OUTPUT);
    
    ledSet(pin, LOW);
}

void ledInit()
{
    if(didInit) {
        return;
    }

    initLed(PIN_LED_RED_L);
    initLed(PIN_LED_GREEN_L);
    initLed(PIN_LED_BLUE_L);
    initLed(PIN_LED_GREEN_R);
    initLed(PIN_LED_RED_R);

    ledSwitchState = LED_LEDSEQ;
    didInit = true;
}

void ledShowFailure(void)
{
  ledSet(ERR_LED1, 1);
  ledSet(ERR_LED2, 1);
}

void ledShowSys(void)
{
    ledSet(SYS_LED, true);
}

bool ledTest(void)
{
    ledSet(PIN_LED_GREEN_L, 1);
    ledSet(PIN_LED_GREEN_R, 1);
    ledSet(PIN_LED_RED_L, 0);
    ledSet(PIN_LED_RED_R, 0);

    delay(250);

    ledSet(PIN_LED_GREEN_L, LOW);
    ledSet(PIN_LED_GREEN_R, LOW);
    ledSet(PIN_LED_RED_L, HIGH);
    ledSet(PIN_LED_RED_R, HIGH);

    delay(250);

    // LED test end
    ledSet(PIN_LED_GREEN_L, LOW);
    ledSet(PIN_LED_GREEN_R, LOW);
    ledSet(PIN_LED_RED_L, LOW);
    ledSet(PIN_LED_RED_R, LOW);
    ledSet(PIN_LED_BLUE_L, HIGH);

    return didInit;
}

void ledSet(const uint8_t pin, bool value)
{
    if (ledSwitchState == LED_LEDSEQ) {
        ledSetForce(pin, value);
    }

    ledLastState[pin] = value;
}

void ledShowFaultPattern(void)
{
    ledSet(PIN_LED_GREEN_L, LOW);
    ledSet(PIN_LED_GREEN_R, LOW);
    ledSet(PIN_LED_RED_L, HIGH);
    ledSet(PIN_LED_RED_R, HIGH);
    ledSet(PIN_LED_BLUE_L, LOW);
}

/**
 * Parameters governing the onboard LEDs
 * */
PARAM_GROUP_START(led)
    /**
     * @brief Control onboard LEDs using a bitmask. Enabling it will override the led sequencer.
     *
     * ```
     * | 7:ENABLE | 6:N/A | 5:BLUE_R | 4:RED_R | 3:GREEN_R | 2:RED_L | 1:GREEN_L | 0:BLUE_L |
     * ```
     */
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, bitmask, &ledControlBitmask, &ledBitmaskParamCallback)

PARAM_GROUP_STOP(led)

