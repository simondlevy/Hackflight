/*
   HackflightSim DynamicsThread implementation

   Copyright(C) 2021 Simon D.Levy

   MIT License
*/

#include "DynamicsThread.h"
#include "hackflight.h"

// Dynamics
static Dynamics * _dynamics;

// Joystick (RC transmitter, game controller) or keypad
static GameInput * _gameInput;
static float _joyvals[4];

// Sent by to stream_writeMotors() -----------
static float _m1;
static float _m2;
static float _m3;
static float _m4;

// Called by Haskell Copilot --------------------------------------------------

void stream_writeMotors(float m1, float m2, float m3, float m4)
{
    _m1 = m1;
    _m2 = m2;
    _m3 = m3;
    _m4 = m4;

    //debugline("m1: %+3.3f  m2: %+3.3f  m3: %+3.3f  m4: %+3.3f", m1, m2, m3, m4);
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


void stream_debug(float value)
{
    debugline("%+3.3f", value);
}

// FDynamicsThread methods -----------------------------------------------------

FDynamicsThread::FDynamicsThread(APawn * pawn, Dynamics * dynamics)
{
    _thread = FRunnableThread::Create(this, TEXT("FThreadedManage"), 0, TPri_BelowNormal); 
    _startTime = FPlatformTime::Seconds();
    _count = 0;

    _dynamics = dynamics;

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
    return _actuatorValues[index];
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

        // Get a current time the OS
        stream_time = FPlatformTime::Seconds() - _startTime;

        // Update dynamics
        _dynamics->update(_actuatorValues, _agl, stream_time);

        // Avoid null-pointer exceptions at startup, freeze after control
        // program halts
        if (!_ready) {
            return 0;
        }

        // Run Copilot, triggering stream_writeMotors
        step();

        // Get state from dynamics
        stream_stateDx     = _dynamics->x(Dynamics::STATE_X_DOT);
        stream_stateDy     = _dynamics->x(Dynamics::STATE_Y_DOT);
        stream_stateDz     = _dynamics->x(Dynamics::STATE_Z_DOT);
        stream_statePhi    = _dynamics->x(Dynamics::STATE_PHI);
        stream_stateDphi   = _dynamics->x(Dynamics::STATE_PHI_DOT);
        stream_stateTheta  = _dynamics->x(Dynamics::STATE_THETA);
        stream_stateDtheta = _dynamics->x(Dynamics::STATE_THETA_DOT);
        stream_statePsi    = _dynamics->x(Dynamics::STATE_PSI);
        stream_stateDpsi   = _dynamics->x(Dynamics::STATE_PSI_DOT);

        // Get updated motor values
        _actuatorValues[0] = _m1;
        _actuatorValues[1] = _m2;
        _actuatorValues[2] = _m3;
        _actuatorValues[3] = _m4;

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
}
