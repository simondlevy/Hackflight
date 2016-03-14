#include "mw.h"
#include "board.h"
#include "config.h" 

#define abs(x) ((x) > 0 ? (x) : -(x))

// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;


int16_t motors[4];
int16_t motor_disarmed[4];

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

void mixerInit(void)
{
    int i;

    // set disarmed motor values
    for (i = 0; i < 4; i++)
        motor_disarmed[i] = CONFIG_MINCOMMAND;
}

void mixerWriteMotors(void)
{
    int16_t maxMotor;
    uint32_t i;

    // prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));

    for (i = 0; i < 4; i++)
        motors[i] = rcCommand[THROTTLE] * mixerQuadX[i].throttle + axisPID[PITCH] * mixerQuadX[i].pitch + 
            axisPID[ROLL] * mixerQuadX[i].roll + -CONFIG_YAW_DIRECTION * axisPID[YAW] * mixerQuadX[i].yaw;

    maxMotor = motors[0];
    for (i = 1; i < 4; i++)
        if (motors[i] > maxMotor)
            maxMotor = motors[i];
    for (i = 0; i < 4; i++) {
        if (maxMotor > CONFIG_MAXTHROTTLE)     
            // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motors[i] -= maxMotor - CONFIG_MAXTHROTTLE;

        motors[i] = constrain(motors[i], CONFIG_MINTHROTTLE, CONFIG_MAXTHROTTLE);
        if ((rcData[THROTTLE]) < CONFIG_MINCHECK) {
            motors[i] = CONFIG_MINTHROTTLE;
        } 
        if (!armed) {
            motors[i] = motor_disarmed[i];
        }
    }

    for (i = 0; i < 4; i++)
        board_writeMotor(i, motors[i]);
}
