#pragma once

#include <assert.h>
#include <console.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <free_rtos.h>
#include <task.h>

#include <clock.hpp>
#include <motors.h>
#include <safety.hpp>
#include <datatypes.h>

class Safety {

    public:

        // Shared with logger
        bool canFlyFlag;
        bool isFlyingFlag;
        bool isTumbledFlag;
        uint16_t infoBitfield;

        // Shared with params
        uint8_t doinfodump;
        uint8_t paramEmergencyStop;
        int8_t deprecatedArmParam;

        void init(void)
        {
            bzero(stateTransitions, sizeof(stateTransitions));

            // Not initialized

            stateTransitions[0][0].newState = statePreFlChecksNotPassed;
            stateTransitions[0][0].triggerCombiner = conditionCombinerAlways;
            stateTransitions[0][0].blockerCombiner = conditionCombinerNever;
            stateTransitions[0][0].next = NULL;

            // Pre-flight checks not passed

            stateTransitions[1][0].newState = stateExceptFreeFall;
            stateTransitions[1][0].triggers = CB_EMERGENCY_STOP;
            stateTransitions[1][0].negatedTriggers = CB_NONE;
            stateTransitions[1][0].triggerCombiner = conditionCombinerAny;
            stateTransitions[1][0].blockerCombiner = conditionCombinerNever;
            stateTransitions[1][0].next = &stateTransitions[1][1];

            stateTransitions[1][1].newState = statePreFlChecksPassed;
            stateTransitions[1][1].triggerCombiner = conditionCombinerAlways;
            stateTransitions[1][1].blockers = CB_CONF_IS_TUMBLED;
            stateTransitions[1][1].negatedBlockers = CB_NONE;
            stateTransitions[1][1].blockerCombiner = conditionCombinerAny;

            // Pre-flight checks passed

            stateTransitions[2][0].newState = stateExceptFreeFall;
            stateTransitions[2][0].triggers = CB_EMERGENCY_STOP;
            stateTransitions[2][0].negatedTriggers = CB_NONE;
            stateTransitions[2][0].triggerCombiner = conditionCombinerAny;
            stateTransitions[2][0].blockerCombiner = conditionCombinerNever;
            stateTransitions[2][0].next = &stateTransitions[2][1];

            stateTransitions[2][1].newState = statePreFlChecksNotPassed;
            stateTransitions[2][1].triggers = CB_CONF_IS_TUMBLED;
            stateTransitions[2][1].negatedTriggers = CB_NONE;
            stateTransitions[2][1].triggerCombiner = conditionCombinerAny;
            stateTransitions[2][1].blockerCombiner = conditionCombinerNever;
            stateTransitions[2][1].next = &stateTransitions[2][2];

            stateTransitions[2][2].newState = stateReadyToFly;
            stateTransitions[2][2].triggers = CB_ARMED;
            stateTransitions[2][2].negatedTriggers = CB_NONE;
            stateTransitions[2][2].triggerCombiner = conditionCombinerAll;
            stateTransitions[2][2].blockers = CB_CONF_IS_TUMBLED;
            stateTransitions[2][2].negatedBlockers = CB_NONE;
            stateTransitions[2][2].blockerCombiner = conditionCombinerAny;

            // Ready to fly

            stateTransitions[3][0].newState = stateExceptFreeFall;
            stateTransitions[3][0].triggers = CB_EMERGENCY_STOP;
            stateTransitions[3][0].negatedTriggers = CB_NONE;
            stateTransitions[3][0].triggerCombiner = conditionCombinerAny;
            stateTransitions[3][0].blockerCombiner = conditionCombinerNever;
            stateTransitions[3][0].next = &stateTransitions[3][1];

            stateTransitions[3][1].newState = statePreFlChecksNotPassed;
            stateTransitions[3][1].triggers = CB_CONF_IS_TUMBLED;
            stateTransitions[3][1].negatedTriggers = CB_ARMED;
            stateTransitions[3][1].triggerCombiner = conditionCombinerAny;
            stateTransitions[3][1].blockerCombiner = conditionCombinerNever;
            stateTransitions[3][1].next = &stateTransitions[3][2];

            stateTransitions[3][2].newState = stateFlying;
            stateTransitions[3][2].triggers = CB_IS_FLYING;
            stateTransitions[3][2].negatedTriggers = CB_NONE;
            stateTransitions[3][2].triggerCombiner = conditionCombinerAll;
            stateTransitions[3][2].blockerCombiner = conditionCombinerNever;

            // Flying

            stateTransitions[4][0].newState = stateExceptFreeFall;
            stateTransitions[4][0].triggers = CB_COMMANDER_WDT_TIMEOUT | 
                CB_CONF_IS_TUMBLED | CB_EMERGENCY_STOP;
            stateTransitions[4][0].negatedTriggers = CB_ARMED;
            stateTransitions[4][0].triggerCombiner = conditionCombinerAny;
            stateTransitions[4][0].blockerCombiner = conditionCombinerNever;
            stateTransitions[4][0].next = &stateTransitions[4][1];

            stateTransitions[4][1].newState = stateWarningLevelOut;
            stateTransitions[4][1].triggers = CB_COMMANDER_WDT_WARNING;
            stateTransitions[4][1].negatedTriggers = CB_NONE;
            stateTransitions[4][1].triggerCombiner = conditionCombinerAll;
            stateTransitions[4][1].blockerCombiner = conditionCombinerNever;
            stateTransitions[4][1].next = &stateTransitions[4][2];

            stateTransitions[4][2].newState = stateLanded;
            stateTransitions[4][2].triggers = CB_NONE;
            stateTransitions[4][2].negatedTriggers = CB_IS_FLYING;
            stateTransitions[4][2].triggerCombiner = conditionCombinerAll;
            stateTransitions[4][2].blockerCombiner = conditionCombinerNever;

            // Landed

            stateTransitions[5][0].newState = stateReset;
            stateTransitions[5][0].triggerCombiner = conditionCombinerAlways;
            stateTransitions[5][0].blockerCombiner = conditionCombinerNever;

            // Reset

            stateTransitions[6][0].newState = statePreFlChecksNotPassed;
            stateTransitions[6][0].triggerCombiner = conditionCombinerAlways;
            stateTransitions[6][0].blockerCombiner = conditionCombinerNever;

            // Warning level-out

            stateTransitions[7][0].newState = stateExceptFreeFall;
            stateTransitions[7][0].triggers = CB_COMMANDER_WDT_TIMEOUT | 
                CB_CONF_IS_TUMBLED | CB_EMERGENCY_STOP;
            stateTransitions[7][0].negatedTriggers = CB_ARMED;
            stateTransitions[7][0].triggerCombiner = conditionCombinerAny;
            stateTransitions[7][0].blockerCombiner = conditionCombinerNever;
            stateTransitions[7][0].next = &stateTransitions[7][1];

            stateTransitions[7][1].newState = stateFlying;
            stateTransitions[7][1].triggers = CB_NONE;
            stateTransitions[7][1].negatedTriggers = CB_COMMANDER_WDT_WARNING | 
                CB_COMMANDER_WDT_TIMEOUT;
            stateTransitions[7][1].triggerCombiner = conditionCombinerAll;
            stateTransitions[7][1].blockerCombiner = conditionCombinerNever;

            // Exception free-fall

            stateTransitions[8][0].newState = stateLocked;
            stateTransitions[8][0].triggerCombiner = conditionCombinerAlways;
            stateTransitions[8][0].blockerCombiner = conditionCombinerNever;

            // Locked

            stateTransitions[9][0].newState = stateLocked;
            stateTransitions[9][0].triggerCombiner = conditionCombinerNever;
            stateTransitions[9][0].blockerCombiner = conditionCombinerAlways;
        }

        bool areMotorsAllowedToRun() 
        {
            return (state == stateReadyToFly) ||
                (state == stateFlying) ||
                (state == stateWarningLevelOut);
        }

        bool canFly() 
        {
            return canFlyFlag;
        }

        bool isFlying() 
        {
            return isFlyingFlag;
        }

        bool isTumbled() 
        {
            return isTumbledFlag;
        }

        bool canArm() 
        {
            return statePreFlChecksPassed == state;
        }

        bool isArmed() 
        {
            return isArmingActivated || deprecatedArmParam;
        }

        bool requestArming(const bool doArm) 
        {
            if (doArm == isArmingActivated) {
                return true;
            }

            if (doArm && !canArm()) {
                return false;
            }

            isArmingActivated = doArm;
            return true;
        }

        void update(
                const sensorData_t & sensors, 
                const uint32_t coreStep,
                const uint32_t timestamp, 
                demands_t & demands)
        { 
            if (!Clock::rateDoExecute(CLOCK_RATE, coreStep)) {
                return;
            }

            const auto currentTick = xTaskGetTickCount();

            const auto conditions = 
                updateAndPopulateConditions(sensors, timestamp, currentTick);

            const auto newState = stateUpdate(state, conditions);

            if (state != newState) {
                const vehicleState_t previousState = state;
                state = newState;
                postTransitionActions(previousState);
            }

            latestConditions = conditions;
            updateLogData(conditions);
            if (doinfodump) {
                doinfodump = 0;
                infoDump();
            }

            // Modify the demands to handle exceptional conditions

            switch(state){

                case stateReadyToFly:
                case stateFlying:
                    break;

                case stateWarningLevelOut:
                    demands.roll = 0;
                    demands.pitch = 0;
                    demands.yaw = 0;
                    // Keep thrust as it is
                    break;

                default:
                    // Replace with zero thrust to stop motors
                    demands.thrust = 0;
                    break;
            }
        }

    private:

        typedef enum {
            conditionArmed = 0,
            conditionIsFlying,
            conditionIsTumbled,
            conditionCommanderWdtWarning,
            conditionCommanderWdtTimeout,
            conditionEmergencyStop,
            condition_NrOfConditions,
        } conditions_t;

        typedef enum {
            stateNotInitialized = 0,
            statePreFlChecksNotPassed,
            statePreFlChecksPassed,
            stateReadyToFly,
            stateFlying,
            stateLanded,
            stateReset,
            stateWarningLevelOut,
            stateExceptFreeFall,
            stateLocked,
            state_NrOfStates,
        } vehicleState_t;

        typedef enum {
            conditionCombinerAll,
            conditionCombinerAny,    
            conditionCombinerAlways,
            conditionCombinerNever,
        } conditionCombiner_t;

        static const Clock::rate_t CLOCK_RATE = Clock::RATE_25_HZ;

        // Condition bit definitions
        static const uint32_t CB_NONE = 0;
        static const uint32_t CB_ARMED = 1 << conditionArmed;
        static const uint32_t CB_IS_FLYING = 1 << conditionIsFlying;
        static const uint32_t CB_IS_TUMBLED = 1 << conditionIsTumbled;
        static const uint32_t CB_COMMANDER_WDT_WARNING = 
            1 << conditionCommanderWdtWarning;
        static const uint32_t CB_COMMANDER_WDT_TIMEOUT = 
            1 << conditionCommanderWdtTimeout;
        static const uint32_t CB_EMERGENCY_STOP = 
            1 << conditionEmergencyStop;


        static const uint32_t CB_CONF_IS_TUMBLED = CB_NONE;

        // The minimum time (in ms) we need to see low thrust before saying that we are
        // not flying anymore
        static const uint32_t IS_FLYING_HYSTERESIS_THRESHOLD = M2T(2000);

        static const uint32_t COMMANDER_WDT_TIMEOUT_STABILIZE = M2T(500);
        static const uint32_t COMMANDER_WDT_TIMEOUT_SHUTDOWN  = M2T(2000);

        static const auto AUTO_ARMING = false;

        typedef struct stateTransition {
            vehicleState_t newState;
            uint32_t triggers;
            uint32_t negatedTriggers;
            conditionCombiner_t triggerCombiner;
            uint32_t blockers;
            uint32_t negatedBlockers;
            conditionCombiner_t blockerCombiner;
            struct stateTransition * next;
        } stateTransition_t;

        char * conditionNames[6] = {
            (char *)"armed",
            (char *)"isFlying",
            (char *)"isTumbled",
            (char *)"commanderWdtWarning",
            (char *)"commanderWdtTimeout",
            (char *)"emergencyStop"
        };

        char * stateNames[10] = {
            (char *)"Not initialized",
            (char *)"Pre-flight checks not passed",
            (char *)"Pre-flight checks passed",
            (char *)"Ready to fly",
            (char *)"Flying",
            (char *)"Landed",
            (char *)"Reset",
            (char *)"Warning, level out",
            (char *)"Exception, free fall",
            (char *)"Locked"
        };


        static bool areAllSet(const uint32_t conditions, 
                const uint32_t requirements) 
        {
            return (~conditions & requirements) == 0;
        }

        static bool isAnySet(
                const uint32_t conditions, 
                const uint32_t requirements) 
        {
            return (conditions & requirements) != 0;
        }

        static bool areConditionsMet(
                const uint32_t conditions, 
                const uint32_t requirements, 
                const uint32_t negRequirements, 
                const conditionCombiner_t combiner) 
        {
            auto result = false;

            switch(combiner) {
                case conditionCombinerAll:
                    result = areAllSet(conditions, requirements) && 
                        !isAnySet(conditions, negRequirements);
                    break;
                case conditionCombinerAny:
                    result = isAnySet(conditions, requirements) ||
                        !areAllSet(conditions, negRequirements); break;
                case conditionCombinerAlways:
                    result = true;
                    break;
                case conditionCombinerNever:
                    result = false;
                    break;
                default:
                    break;
            }

            return result;
        }

        stateTransition_t stateTransitions[10][3];

        bool isArmingActivated;
        uint32_t initialTumbleTick;
        uint32_t latestThrustTick;
        vehicleState_t state;
        uint32_t latestConditions;

        vehicleState_t findTransition(
                const vehicleState_t currentState, 
                const uint32_t conditions, 
                const stateTransition * transitions) 
        {
            vehicleState_t newState = currentState;

            for (auto transition=transitions; transition; transition=transition->next) {

                const auto isTriggerMatch =
                    areConditionsMet(conditions, transition->triggers, 
                            transition->negatedTriggers, 
                            transition->triggerCombiner);

                const auto isBlockerMatch =
                    areConditionsMet(conditions, transition->blockers, 
                            transition->negatedBlockers, 
                            transition->blockerCombiner);

                const auto isStateTransitionValid = isTriggerMatch && !isBlockerMatch;

                if (isStateTransitionValid) {
                    newState = transition->newState;
                    break;
                }
            }

            return newState;
        }

        //
        // We say we are flying if one or more motors are running over the idle thrust.
        //
        bool isFlyingCheck(const uint32_t tick) 
        {
            auto isThrustOverIdle = false;

            for (int i = 0; i < 4; ++i) {
                if (motorsGetRatio(i) > 0) {
                    isThrustOverIdle = true;
                    break;
                }
            }

            if (isThrustOverIdle) {
                latestThrustTick = tick;
            }

            bool result = false;
            if (0 != latestThrustTick) {
                if ((tick - latestThrustTick) < IS_FLYING_HYSTERESIS_THRESHOLD) {
                    result = true;
                }
            }

            return result;
        }

        bool isTumbledCheck(const sensorData_t & sensors, const uint32_t tick) 
        {
            const float freeFallThreshold = 0.1;

            const float acceptedTiltAccZ = 0.5;  // 60 degrees tilt (when stationary)

            const auto maxTiltTime = M2T(1000);

            const float acceptedUpsideDownAccZ = -0.2;

            const auto maxUpsideDownTime = M2T(100);

            const auto isFreeFalling = 
                (fabsf(sensors.acc.z) < freeFallThreshold && 
                 fabsf(sensors.acc.y) < freeFallThreshold && fabsf(sensors.acc.x) <
                 freeFallThreshold); 

            if (isFreeFalling) {
                // Falling is OK, reset
                initialTumbleTick = 0;
            }

            const bool isTilted = (sensors.acc.z < acceptedTiltAccZ);

            if(isTilted) {  // Will also be true for up side down
                if (0 == initialTumbleTick) {
                    // Start the clock
                    initialTumbleTick = tick;
                }

                const auto ticksBeingTumbled = tick - initialTumbleTick;

                const auto isUpSideDown = (sensors.acc.z < acceptedUpsideDownAccZ);
                if (isUpSideDown && (ticksBeingTumbled > maxUpsideDownTime)) {
                    return true;
                }

                if (ticksBeingTumbled > maxTiltTime) {
                    return true;
                }
            } else {
                // We're OK, reset
                initialTumbleTick = 0;
            }

            return false;
        }

        vehicleState_t stateUpdate(const vehicleState_t currentState, const uint32_t conditions) 
        {
            const auto transitions = &(stateTransitions[currentState][0]);

            return findTransition(currentState, conditions, transitions);
        }


        const char* getStateName(const vehicleState_t state) 
        {
            return stateNames[state];
        }

        const char* getConditionName(const vehicleState_t condition) 
        {
            return conditionNames[condition];
        }

        void infoDump(void) 
        {
            consolePrintf("SAFETY: Safety info ---\n");
            consolePrintf("SAFETY: State: %s\n", getStateName(state));
            consolePrintf("SAFETY: Conditions: (0x%lx)\n", latestConditions);

            for (uint8_t condition = 0; condition < condition_NrOfConditions;
                    condition++) { 
                const uint32_t bit = 1 << condition;
                int bitValue = 0;
                if (latestConditions & bit) {
                    bitValue = 1;
                }

                consolePrintf("SAFETY:   %s (0x%lx): %u\n",
                        getConditionName((vehicleState_t)condition), 
                        bit, bitValue);
            }
        }

        void postTransitionActions(const vehicleState_t previousState) 
        {
            const auto newState = state;

            if (newState == stateReadyToFly) {
                consolePrintf("SAFETY: Ready to fly\n");
            }

            if (newState == stateLocked) {
                consolePrintf("SAFETY: Locked, reboot required\n");
            }

            if ((previousState == stateNotInitialized || 
                        previousState == stateReadyToFly || 
                        previousState == stateFlying) &&
                    newState != stateReadyToFly && 
                    newState != stateFlying) {

                consolePrintf("SAFETY: Can not fly\n");
            }

            if (newState != stateReadyToFly &&
                    newState != stateFlying &&
                    newState != stateWarningLevelOut) {
                requestArming(false);
            }

            // We do not require an arming action by the user, auto arm
            if (AUTO_ARMING || deprecatedArmParam) {
                if (newState == statePreFlChecksPassed) {
                    requestArming(true);
                }
            }
        }

        uint32_t updateAndPopulateConditions(
                const sensorData_t & sensors, 
                const uint32_t timestamp,
                const uint32_t currentTick) 
        {
            uint32_t conditions = 0;

            if (isArmed()) {
                conditions |= CB_ARMED;
            }

            const auto isFlying = isFlyingCheck(currentTick);

            if (isFlying) {
                conditions |= CB_IS_FLYING;
            }

            const auto isTumbled = isTumbledCheck(sensors, currentTick);

            if (isTumbled) {
                conditions |= CB_IS_TUMBLED;
            }

            const auto age = currentTick - timestamp;

            if (age > COMMANDER_WDT_TIMEOUT_STABILIZE) {
                conditions |= CB_COMMANDER_WDT_WARNING;
            }
            if (age > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
                conditions |= CB_COMMANDER_WDT_TIMEOUT;
            }

            if (paramEmergencyStop) {
                conditions |= CB_EMERGENCY_STOP;
            }

            return conditions;
        }

        void updateLogData(const uint32_t conditions) 
        {
            canFlyFlag = areMotorsAllowedToRun();

            isFlyingFlag = (state == stateFlying) || 
                (state == stateWarningLevelOut);

            isTumbledFlag = (conditions & CB_IS_TUMBLED) != 0;

            infoBitfield = 0;
            if (canArm()) {
                infoBitfield |= 0x0001;
            }
            if (isArmed()) {
                infoBitfield |= 0x0002;
            }
            if(AUTO_ARMING || deprecatedArmParam) {
                infoBitfield |= 0x0004;
            }
            if (canFlyFlag) {
                infoBitfield |= 0x0008;
            }
            if (isFlyingFlag) {
                infoBitfield |= 0x0010;
            }
            if (isTumbledFlag) {
                infoBitfield |= 0x0020;
            }
        }
};

