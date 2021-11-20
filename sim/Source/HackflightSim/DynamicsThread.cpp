/*
   HackflightSim DynamicsThread implementation

   Copyright(C) 2021 Simon D.Levy

   MIT License
*/


#include "DynamicsThread.h"
#include "hackflight.h"

// Joystick (RC transmitter, game controller) or keypad
static GameInput * _gameInput;
static float _joyvals[4];

// Called by Haskell Copilot --------------------------------------------------

static float _m1;
static float _m2;
static float _m3;
static float _m4;
static float _m5;
static float _m6;
static float _m7;
static float _m8;

void stream_setMotorsQuad(float m1, float m2, float m3, float m4)
{
    _m1 = m1;
    _m2 = m2;
    _m3 = m3;
    _m4 = m4;
}

static float _x;
static float _y;
static float _z;
static float _phi;
static float _theta;
static float _psi;

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


// FDynamicsThread methods -----------------------------------------------------

FDynamicsThread::FDynamicsThread(APawn * pawn)
{
    _thread = FRunnableThread::Create(this, TEXT("FThreadedManage"), 0, TPri_BelowNormal); 
    _startTime = FPlatformTime::Seconds();

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

float FDynamicsThread::getActuatorValue(uint8_t index)
{
    return _motorValues[index];
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
        _motorValues[4] = _m5;
        _motorValues[5] = _m6;
        _motorValues[6] = _m7;
        _motorValues[7] = _m8;
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
