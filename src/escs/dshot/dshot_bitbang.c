/*
   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>
#include <string.h>

#include <maxmotors.h>
#include <time.h>

#include "platform.h"

#include "atomic.h"
#include "dma.h"
#include "dma_reqmap.h"
#include "escs/dshot_command.h"
#include "escs/dshot_dev.h"
#include "dshot_bitbang.h"
#include "io.h"
#include "io_impl.h"
#include "nvic.h"
#include "systemdev.h"
#include "timer.h"

#define MAX_MOTOR_PACERS  4

// Max direct dshot port groups, limited by number of usable timer (TIM1 and
// TIM8) x number of channels per timer (4), 3 is enough to cover motor pins on
// GPIOA, GPIOB and GPIOC.
#define MAX_SUPPORTED_MOTOR_PORTS 4 

static uint32_t TELEMETRY_OVER_SAMPLE = 3;

#define BIT_PER_SYMBOL 1

#define STATE_PER_SYMBOL 3

#define FRAME_BITS 16

#define BUF_LENGTH ((FRAME_BITS / BIT_PER_SYMBOL) * STATE_PER_SYMBOL)

// DMA input buffer
// (30us + <frame time> + <slack>) / <input sampling clock period>
// <frame time> = <DShot symbol time> * 16
// Temporary size for DS600
// <frame time> = 26us
// <sampling period> = 0.44us
// <slack> = 10%
// (30 + 26 + 3) / 0.44 = 134
// In some cases this was not enough, so we add 6 extra samples
#define PORT_IP_BUF_LENGTH 140

typedef struct bbMotor_s {
    dshotProtocolControl_t protocolControl;
    int pinIndex;    // pinIndex of this motor output within a group that bbPort points to
    int portIndex;
    IO_t io;         // IO_t for this output
    uint8_t output;
    uint32_t iocfg;
    bbPort_t *bbPort;
    bool configured;
    bool enabled;
} motor_t;

typedef enum {
    TIMER_AUTO = 0,
    TIMER_TIM1,
    TIMER_TIM8,
} timer_e;

typedef struct {
    volatile timCCR_t *ccr;
    TIM_TypeDef       *tim;
} timerChannel_t;

typedef struct {
    timerChannel_t channel;
    float pulseScale;
    float pulseOffset;
    bool forceOverflow;
    bool enabled;
    IO_t io;
} pwmOutputPort_t;

static timer_e USE_TIMER = TIMER_AUTO;

static bbPacer_t m_pacers[MAX_MOTOR_PACERS];  // TIM1 or TIM8
static int m_usedMotorPacers = 0;

static bbPort_t m_ports[MAX_SUPPORTED_MOTOR_PORTS];
static int m_usedMotorPorts;

static motor_t m_motors[MAX_SUPPORTED_MOTORS];

static int m_motorCount;

static uint32_t m_outputBuffer[BUF_LENGTH * MAX_SUPPORTED_MOTOR_PORTS];
static uint16_t m_inputBuffer[PORT_IP_BUF_LENGTH * MAX_SUPPORTED_MOTOR_PORTS];

static uint8_t m_puPdMode;

uint32_t m_frameUs;

const timerHardware_t m_timerHardware[] = {
    DEF_TIM(TIM1,  CH1, NONE,  TIM_USE_NONE, 0, 1),
    DEF_TIM(TIM1,  CH1, NONE,  TIM_USE_NONE, 0, 2),
    DEF_TIM(TIM1,  CH2, NONE,  TIM_USE_NONE, 0, 1),
    DEF_TIM(TIM1,  CH3, NONE,  TIM_USE_NONE, 0, 1),
    DEF_TIM(TIM1,  CH4, NONE,  TIM_USE_NONE, 0, 0),
};

static uint32_t m_lastSendUs;

// DMA GPIO output buffer formatting

static void outputDataInit(uint32_t *buffer, uint16_t portMask, bool inverted)
{
    uint32_t resetMask;
    uint32_t setMask;

    if (inverted) {
        resetMask = portMask;
        setMask = (portMask << 16);
    } else {
        resetMask = (portMask << 16);
        setMask = portMask;
    }

    int bitpos;

    for (bitpos = 0; bitpos < 16; bitpos++) {
        buffer[bitpos * 3 + 0] |= setMask ; // Always set all ports
        buffer[bitpos * 3 + 1] = 0;          // Reset bits are port dependent
        buffer[bitpos * 3 + 2] |= resetMask; // Always reset all ports
    }
}

static void outputDataSet(uint32_t *buffer, int pinNumber, uint16_t value, bool inverted)
{
    uint32_t middleBit;

    if (inverted) {
        middleBit = (1 << (pinNumber + 0));
    } else {
        middleBit = (1 << (pinNumber + 16));
    }

    for (int pos = 0; pos < 16; pos++) {
        if (!(value & 0x8000)) {
            buffer[pos * 3 + 1] |= middleBit;
        }
        value <<= 1;
    }
}

static void outputDataClear(uint32_t *buffer)
{
    // Middle position to no change
    for (int bitpos = 0; bitpos < 16; bitpos++) {
        buffer[bitpos * 3 + 1] = 0;
    }
}

static bbPacer_t *findMotorPacer(TIM_TypeDef *tim)
{
    for (int i = 0; i < MAX_MOTOR_PACERS; i++) {

        bbPacer_t *bbPacer = &m_pacers[i];

        if (bbPacer->tim == NULL) {
            bbPacer->tim = tim;
            ++m_usedMotorPacers;
            return bbPacer;
        }

        if (bbPacer->tim == tim) {
            return bbPacer;
        }
    }

    return NULL;
}

// bbPort management

static bbPort_t *findMotorPort(int portIndex)
{
    for (int i = 0; i < m_usedMotorPorts; i++) {
        if (m_ports[i].portIndex == portIndex) {
            return &m_ports[i];
        }
    }
    return NULL;
}

static bbPort_t *allocateMotorPort(int portIndex)
{
    if (m_usedMotorPorts >= MAX_SUPPORTED_MOTOR_PORTS) {
        return NULL;
    }

    bbPort_t *bbPort = &m_ports[m_usedMotorPorts];

    if (!bbPort->timhw) {
        // No more pacer channel available
        return NULL;
    }

    bbPort->portIndex = portIndex;
    bbPort->owner.owner = OWNER_DSHOT_BITBANG;
    bbPort->owner.resourceIndex = RESOURCE_INDEX(portIndex);

    ++m_usedMotorPorts;

    return bbPort;
}

// Return frequency of smallest change [state/sec]

static uint32_t getDshotBaseFrequency(dshotProtocol_t pwmProtocolType)
{
    switch (pwmProtocolType) {
        case(DSHOT600):
            return 600;
        case(DSHOT300):
            return 300;
        default:
        case(DSHOT150):
            return 150;
    }
}

static void setupDma(bbPort_t *bbPort)
{
    const dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(bbPort->dmaResource);
    dmaEnable(dmaIdentifier);
    bbPort->dmaSource = timerDmaSource(bbPort->timhw->channel);

    bbPacer_t *bbPacer = findMotorPacer(bbPort->timhw->tim);
    bbPacer->dmaSources |= bbPort->dmaSource;

    dmaSetHandler(
            dmaIdentifier,
            bbDMAIrqHandler,
            NVIC_BUILD_PRIORITY(2, 1),
            (uint32_t)bbPort);

    bbDMA_ITConfig(bbPort);
}

static const resourceOwner_t *timerGetOwner(const timerHardware_t *timer)
{
    for (int index = 0; index < m_usedMotorPorts; index++) {
        const timerHardware_t *bitbangTimer = m_ports[index].timhw;
        if (bitbangTimer && bitbangTimer == timer) {
            return &m_ports[index].owner;
        }
    }

    return &freeOwner;
}

static const timerHardware_t *timerGetAllocatedByNumberAndChannel(
        int8_t timerNumber,
        uint16_t timerChannel)
{
    for (int index = 0; index < m_usedMotorPorts; index++) {
        const timerHardware_t *bitbangTimer = m_ports[index].timhw;
        if (bitbangTimer && timerGetTIMNumber(bitbangTimer->tim) == timerNumber
                && bitbangTimer->channel == timerChannel &&
                m_ports[index].owner.owner) { return bitbangTimer;
        }
    }

    return NULL;
}


// Setup m_ports array elements so that they each have a TIM1 or TIM8 channel
// in timerHardware array for BB-DShot.

static void findPacerTimer(void)
{
    for (int bbPortIndex=0; bbPortIndex<MAX_SUPPORTED_MOTOR_PORTS; bbPortIndex++) {

        for (uint8_t tmrIndex=0; tmrIndex<ARRAYLEN(m_timerHardware); tmrIndex++) {
            const timerHardware_t *timer = &m_timerHardware[tmrIndex];
            int timNumber = timerGetTIMNumber(timer->tim);
            if ((USE_TIMER == TIMER_TIM1 && timNumber != 1)
                    || (USE_TIMER == TIMER_TIM8 &&
                        timNumber != 8)) {
                continue;
            }
            bool timerConflict = false;
            for (int channel = 0; channel < CC_CHANNELS_PER_TIMER; channel++) {
                const timerHardware_t *timer = timerGetAllocatedByNumberAndChannel(
                        timNumber, CC_CHANNEL_FROM_INDEX(channel)); 
                const resourceOwner_e timerOwner = timerGetOwner(timer)->owner;
                if (timerOwner != OWNER_FREE && timerOwner != OWNER_DSHOT_BITBANG) {
                    timerConflict = true;
                    break;
                }
            }

            for (int index = 0; index < bbPortIndex; index++) {
                const timerHardware_t* t = m_ports[index].timhw;
                if (timerGetTIMNumber(t->tim) == timNumber &&
                        timer->channel == t->channel) {
                    timerConflict = true;
                    break;
                }
            }

            if (timerConflict) {
                continue;
            }

            dmaoptValue_t dmaopt = dmaGetOptionByTimer(timer);
            const dmaChannelSpec_t *dmaChannelSpec =
                dmaGetChannelSpecByTimerValue(timer->tim, timer->channel,
                        dmaopt); dmaResource_t *dma = dmaChannelSpec->ref;
            dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dma);
            if (dmaGetOwner(dmaIdentifier)->owner == OWNER_FREE) {
                m_ports[bbPortIndex].timhw = timer;

                break;
            }
        }
    }
}

static void timebaseSetup(bbPort_t *bbPort, dshotProtocol_t dshotProtocolType)
{
    uint32_t timerclock = timerClock(bbPort->timhw->tim);

    uint32_t outputFreq = 1000 * getDshotBaseFrequency(dshotProtocolType);
    m_frameUs = 1000000 * 17 * 3 / outputFreq;
    bbPort->outputARR = timerclock / outputFreq - 1;

    uint32_t inputFreq = outputFreq * 5 * 2 * TELEMETRY_OVER_SAMPLE / 24;
    bbPort->inputARR = timerclock / inputFreq - 1;
}

// bb only use pin info associated with timerHardware entry designated as
// TIM_USE_MOTOR; it does not use the timer channel associated with the pin.

static bool motorConfig(
        IO_t io, uint8_t motorIndex, dshotProtocol_t pwmProtocolType, uint8_t output)
{
    int pinIndex = IO_GPIOPinIdx(io);
    int portIndex = IO_GPIOPortIdx(io);

    bbPort_t *bbPort = findMotorPort(portIndex);

    if (!bbPort) {

        // New port group

        bbPort = allocateMotorPort(portIndex);

        if (bbPort) {
            const timerHardware_t *timhw = bbPort->timhw;
            const dmaChannelSpec_t *dmaChannelSpec =
                dmaGetChannelSpecByTimerValue(timhw->tim, timhw->channel,
                        dmaGetOptionByTimer(timhw)); bbPort->dmaResource =
                dmaChannelSpec->ref;
            bbPort->dmaChannel = dmaChannelSpec->channel;
        }

        bbPort->gpio = IO_GPIO(io);

        bbPort->portOutputCount = BUF_LENGTH;
        bbPort->portOutputBuffer = &m_outputBuffer[(bbPort - m_ports) * BUF_LENGTH];

        bbPort->portInputCount = PORT_IP_BUF_LENGTH;
        bbPort->portInputBuffer = &m_inputBuffer[(bbPort - m_ports) * PORT_IP_BUF_LENGTH];

        timebaseSetup(bbPort, pwmProtocolType);
        bbTIM_TimeBaseInit(bbPort, bbPort->outputARR);
        bbTimerChannelInit(bbPort, OWNER_DSHOT_BITBANG);

        setupDma(bbPort);
        bbDMAPreconfigure(bbPort, BITBANG_DIRECTION_OUTPUT);
        bbDMAPreconfigure(bbPort, BITBANG_DIRECTION_INPUT);

        bbDMA_ITConfig(bbPort);
    }

    motor_t * bbMotor = &m_motors[motorIndex];

    bbMotor->pinIndex = pinIndex;
    bbMotor->io = io;
    bbMotor->output = output;
    bbMotor->bbPort = bbPort;

    IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

    // Setup GPIO_MODER and GPIO_ODR register manipulation values

    bbGpioSetup(bbMotor->bbPort, bbMotor->pinIndex, bbMotor->io, m_puPdMode);

    outputDataInit(bbPort->portOutputBuffer, (1 << pinIndex), false); // not inverted

    bbSwitchToOutput(bbPort);

    bbMotor->configured = true;

    return true;
}

// -------------------------------------------------------------------------

void dshotBitbangDevInit(const uint8_t pins[], const uint8_t count)
{
    m_motorCount = count;

    memset(m_outputBuffer, 0, sizeof(m_outputBuffer));

    for (int motorIndex = 0; motorIndex < m_motorCount; motorIndex++) {
        const timerHardware_t *timerHardware = timerGetConfiguredByTag(pins[motorIndex]);
        const IO_t io = IOGetByTag(pins[motorIndex]);

        uint8_t output = timerHardware->output;
        m_puPdMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PuPd_DOWN : GPIO_PuPd_UP;

        if (!IOIsFreeOrPreinit(io)) {
            // not enough motors initialised for the mixer or a break in the motors
            return NULL;
        }

        int pinIndex = IO_GPIOPinIdx(io);

        m_motors[motorIndex].pinIndex = pinIndex;
        m_motors[motorIndex].io = io;
        m_motors[motorIndex].output = output;
        m_motors[motorIndex].iocfg = IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz,
                GPIO_OType_PP, m_puPdMode);

        IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
        IOConfigGPIO(io, m_motors[motorIndex].iocfg);
        if (output & TIMER_OUTPUT_INVERTED) {
            IOLo(io);
        } else {
            IOHi(io);
        }
    }
}

bool dshotBitbangEnableMotors(void)
{
    for (int i = 0; i < m_motorCount; i++) {
        if (m_motors[i].configured) {
            IOConfigGPIO(m_motors[i].io, m_motors[i].iocfg);
        }
    }
    return true;
}

void dshotBitbangPostInit(dshotProtocol_t protocol)
{
    findPacerTimer();

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex <
            m_motorCount; motorIndex++) {

        if (!motorConfig(m_motors[motorIndex].io, motorIndex,
                    protocol, m_motors[motorIndex].output)) { 
            return NULL;
        }


        m_motors[motorIndex].enabled = true;
    }
}

void dshotBitbangUpdateComplete(uint8_t motorCount)
{
    // If there is a dshot command loaded up, time it correctly with motor update

    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(motorCount)) {
            return;
        }
    }

    for (int i = 0; i < m_usedMotorPorts; i++) {
        bbPort_t *bbPort = &m_ports[i];

        bbDMA_Cmd(bbPort, ENABLE);
    }

    m_lastSendUs = micros();
    for (int i = 0; i < m_usedMotorPacers; i++) {
        bbPacer_t *bbPacer = &m_pacers[i];
        bbTIM_DMACmd(bbPacer->tim, bbPacer->dmaSources, ENABLE);
    }
}

bool dshotBitbangUpdateStart(void)
{
    for (int i = 0; i < m_usedMotorPorts; i++) {
        bbDMA_Cmd(&m_ports[i], DISABLE);
        outputDataClear(m_ports[i].portOutputBuffer);
    }

    return true;
}

void dshotBitbangWrite(uint8_t motorIndex, float value)
{
    uint16_t ivalue = (uint16_t)value;

    motor_t *const bbmotor = &m_motors[motorIndex];

    if (!bbmotor->configured) {
        return;
    }

    // fetch requestTelemetry from motors. Needs to be refactored.
    motorDmaOutput_t * const motor = getMotorDmaOutput(motorIndex);
    bbmotor->protocolControl.requestTelemetry =
        motor->protocolControl.requestTelemetry;
    motor->protocolControl.requestTelemetry = false;

    // If there is a command ready to go overwrite the value and send that instead
    if (dshotCommandIsProcessing()) {
        ivalue = dshotCommandGetCurrent(motorIndex);
        if (ivalue) {
            bbmotor->protocolControl.requestTelemetry = true;
        }
    }

    bbmotor->protocolControl.value = ivalue;

    uint16_t packet = prepareDshotPacket(&bbmotor->protocolControl);

    bbPort_t *bbPort = bbmotor->bbPort;

    outputDataSet( bbPort->portOutputBuffer, bbmotor->pinIndex, packet, false); 
}


