extern "C" {

#include "mw.hpp"

// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

void Mixer::init(RC * rc, PID * pid)
{
    this->_pid = pid;
    this->_rc = rc;

    // set disarmed motor values
    for (uint8_t i = 0; i < 4; i++)
        this->motorsDisarmed[i] = CONFIG_MINCOMMAND;
}

void Mixer::update(bool armed)
{
    int16_t maxMotor;
    int16_t motors[4];

    for (uint8_t i = 0; i < 4; i++)
        motors[i] = this->_rc->command[THROTTLE] * mixerQuadX[i].throttle + this->_pid->axisPID[PITCH] * mixerQuadX[i].pitch + 
            this->_pid->axisPID[ROLL] * mixerQuadX[i].roll + -CONFIG_YAW_DIRECTION * this->_pid->axisPID[YAW] * mixerQuadX[i].yaw;

    maxMotor = motors[0];

    for (uint8_t i = 1; i < 4; i++)
        if (motors[i] > maxMotor)
            maxMotor = motors[i];

    for (uint8_t i = 0; i < 4; i++) {

        if (maxMotor > CONFIG_MAXTHROTTLE)     
            // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motors[i] -= maxMotor - CONFIG_MAXTHROTTLE;

        motors[i] = constrain(motors[i], CONFIG_MINTHROTTLE, CONFIG_MAXTHROTTLE);

        if (this->_rc->throttleIsDown()) {
            motors[i] = CONFIG_MINTHROTTLE;
        } 

        if (!armed) {
            motors[i] = motorsDisarmed[i];
        }
    }

    for (uint8_t i = 0; i < 4; i++)
        board_writeMotor(i, motors[i]);
}

} // extern "C"
