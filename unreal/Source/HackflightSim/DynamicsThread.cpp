/*
   HackflightSim DynamicsThread implementation

   Copyright(C) 2021 Simon D.Levy

   MIT License
*/


#include "DynamicsThread.h"
#include "hackflight.h"

#include "../../../dynamics/Dynamics.hpp"

// Joystick (RC transmitter, game controller) or keypad
static GameInput * _gameInput;
static float _joyvals[4];

// Sent by stream_setMotors() -----------
static float _m1;
static float _m2;
static float _m3;
static float _m4;

// Sent by stream_setPose() -----------
static float _x;
static float _y;
static float _z;
static float _phi;
static float _theta;
static float _psi;

// XXX Debugging -------------------------------------------------------------

Dynamics::vehicle_params_t vparams = {

    // Estimated
    2.E-06, // d drag cofficient [T=d*w^2]

    // https://www.dji.com/phantom-4/info
    1.380,  // m mass [kg]

    // Estimated
    2,      // Ix [kg*m^2] 
    2,      // Iy [kg*m^2] 
    3,      // Iz [kg*m^2] 
    3.8E-03, // Jr prop inertial [kg*m^2] 
    15000,// maxrpm
};

Dynamics::fixed_pitch_params_t fpparams = {
    5.E-06, // b thrust coefficient [F=b*w^2]
    0.350   // l arm length [m]
};

Dynamics _dynamics = Dynamics(vparams, fpparams);

static float _cppval;

// Called by Haskell Copilot --------------------------------------------------

void stream_setMotors(float m1, float m2, float m3, float m4)
{
    _m1 = m1;
    _m2 = m2;
    _m3 = m3;
    _m4 = m4;

    // debugline("m1: %+3.3f  m2: %+3.3f  m3: %+3.3f  m4: %+3.3f", m1, m2, m3, m4);
}

void stream_setPose(float x, float y, float z, float phi, float theta, float psi)
{
    _x = x;
    _y = y;
    _z = z;
    _phi = phi;
    _theta = theta;
    _psi = psi;
}

void stream_getReceiverDemands(void)
{
    // Get stick demands
    _gameInput->getJoystick(_joyvals);

    // Share the stick demands
    stream_receiverThrottle = _joyvals[0];
    stream_receiverRoll     = _joyvals[1];
    stream_receiverPitch    = _joyvals[2];
    stream_receiverYaw      = _joyvals[3];
}


void stream_debug(float hsval)
{
    debugline("%f   %f | %3.3f", hsval, _cppval, abs(hsval-_cppval));
}

// FDynamicsThread methods -----------------------------------------------------

FDynamicsThread::FDynamicsThread(APawn * pawn)
{
    _thread = FRunnableThread::Create(this, TEXT("FThreadedManage"), 0, TPri_BelowNormal); 
    _startTime = FPlatformTime::Seconds();
    _count = 0;

    _gameInput = new GameInput(pawn);

    _ready = true;
}

FDynamicsThread::~FDynamicsThread()
{
    delete _thread;
}

void FDynamicsThread::tick(void)
{
    // Get demands from keypad
    _gameInput->getKeypad(_joyvals);
}

float FDynamicsThread::actuatorValue(uint8_t index)
{
    return _motorValues[index];
}

uint32_t FDynamicsThread::getCount(void)
{
    return _count;
}

void FDynamicsThread::stopThread(FDynamicsThread ** worker)
{
    if (*worker) {
        (*worker)->Stop();
        delete *worker;
    }

    *worker = NULL;
}

bool FDynamicsThread::Init() 
{
    _running = false;

    return FRunnable::Init();
}

uint32_t FDynamicsThread::Run()
{
    // Initial wait before starting
    FPlatformProcess::Sleep(0.5);

    _running = true;

    while (_running) {

        // Get current time from the OS, sharing it with Haskell
        stream_time = FPlatformTime::Seconds() - _startTime;

        // Avoid null-pointer exceptions at startup, freeze after control
        // program halts
        if (!_ready) {
            return 0;
        }

        // Run Copilot, triggering stream_writeMotors
        step();

        // Get updated motor values
        _motorValues[0] = _m1;
        _motorValues[1] = _m2;
        _motorValues[2] = _m3;
        _motorValues[3] = _m4;

        Dynamics::state_t state = {};
        _dynamics.update(_motorValues, state, _agl, stream_time, _cppval);

        // Increment count for FPS reporting
        _count++;
    }

    return 0;
}

void FDynamicsThread::Stop()
{
    _running = false;

    // Final wait after stopping
    FPlatformProcess::Sleep(0.03);

    FRunnable::Stop();
}

void FDynamicsThread::setAgl(float agl)
{
    _agl = agl;
    stream_agl = agl;
}


void FDynamicsThread::getPose(float & x, float & y, float & z, float & phi, float & theta, float & psi)
{
    x = _x;
    y = _y;
    z = _z;
    phi   = _phi;
    theta = _theta;
    psi   = _psi;
}
