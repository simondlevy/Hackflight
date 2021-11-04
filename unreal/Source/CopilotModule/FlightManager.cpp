/*
   MulticopterSim FlightManager class implementation using Haskell Copilot

   Copyright(C) 2021 Simon D.Levy

   MIT License
*/

#include "FlightManager.h"

#include "copilot.h"

// Sent by Copilot to copilot_runMotors() -----------
static float _m1;
static float _m2;
static float _m3;
static float _m4;

// Called by Copilot
void copilot_writeMotors(float m1, float m2, float m3, float m4)
{
    _m1 = m1;
    _m2 = m2;
    _m3 = m3;
    _m4 = m4;

    //debugline("m1: %+3.3f  m2: %+3.3f  m3: %+3.3f  m4: %+3.3f", m1, m2, m3, m4);
}

void copilot_debug(float value)
{
    debugline("%+3.3f", value);
}


FCopilotFlightManager::FCopilotFlightManager(APawn * pawn, Dynamics * dynamics)
    : FFlightManager(dynamics)
{
    _gameInput = new GameInput(pawn);

    _ready = true;
}

FCopilotFlightManager::~FCopilotFlightManager()
{
}

void FCopilotFlightManager::getReceiverDemands(void)
{
    // Get stick demands
    _gameInput->getJoystick(_joyvals);

    // Share the stick demands
    copilot_receiverThrottle = _joyvals[0];
    copilot_receiverRoll     = _joyvals[1];
    copilot_receiverPitch    = _joyvals[2];
    copilot_receiverYaw      = _joyvals[3];
}


void FCopilotFlightManager::getGyrometer(void)
{
    copilot_gyrometerX = FMath::RadiansToDegrees(_dynamics->x(Dynamics::STATE_PHI_DOT)); 
    copilot_gyrometerY = FMath::RadiansToDegrees(_dynamics->x(Dynamics::STATE_THETA_DOT)); 
    copilot_gyrometerZ = FMath::RadiansToDegrees(_dynamics->x(Dynamics::STATE_PSI_DOT)); 
}

void FCopilotFlightManager::getQuaternion(void)
{
    FRotator rot = FRotator(
            FMath::RadiansToDegrees(_dynamics->x(Dynamics::STATE_THETA)),
            FMath::RadiansToDegrees(_dynamics->x(Dynamics::STATE_PSI)),
            FMath::RadiansToDegrees(_dynamics->x(Dynamics::STATE_PHI))
            );

    FQuat quat = rot.Quaternion();

    copilot_quaternionW = quat.W;
    copilot_quaternionX = quat.X;
    copilot_quaternionY = quat.Y;
    copilot_quaternionZ = quat.Z;
}

void FCopilotFlightManager::getOpticalFlow(void)
{
    double dx = _dynamics->x(Dynamics::STATE_X_DOT);
    double dy = _dynamics->x(Dynamics::STATE_Y_DOT);

    double psi = _dynamics->x(Dynamics::STATE_PSI);
    double cp = cos(psi);
    double sp = sin(psi);

    // Rotate inertial velocity into body frame, ignoring roll and pitch fow now
    copilot_flowX = dx * cp + dy * sp;
    copilot_flowY = dy * cp - dx * sp;
}

void FCopilotFlightManager::getActuators(const double time, double * values)
{
    // Avoid null-pointer exceptions at startup, freeze after control
    // program halts
    if (!_ready) {
        return;
    }

    // Share the current time with Copilot
    copilot_time = time; 

    // Share stick demands with Copilot
    getReceiverDemands();

    // Share the gyrometer values
    getGyrometer();

    // Share the quaternion values
    getQuaternion();

    // Share the optical flow values
    getOpticalFlow();

    // Share the altimeter value
    copilot_altimeterZ = _dynamics->x(Dynamics::STATE_Z); 

    // Run Copilot, triggering copilot_runMotors
    step();

    // Get updated motor values
    values[0] = _m1;
    values[1] = _m2;
    values[2] = _m3;
    values[3] = _m4;
}

void FCopilotFlightManager::tick(void)
{
    // Get demands from keypad
    _gameInput->getKeypad(_joyvals);
}
