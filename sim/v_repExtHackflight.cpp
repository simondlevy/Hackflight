/*
   V-REP simulator plugin code for Hackflight

   Copyright (C) Simon D. Levy 2016

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include "v_repExtHackflight.hpp"
#include "scriptFunctionData.h"
#include "v_repLib.h"

// From firmware
extern void setup(void);
extern void loop(void);

#define JOY_DEV "/dev/input/js0"

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

#define PLUGIN_NAME "Hackflight"

static LIBRARY vrepLib;

class LED {

    private:

        int handle;
        float color[3];
        bool on;
        float black[3];

    public:

        LED(void) {}

        LED(int handle, int r, int g, int b) {
            this->handle = handle;
            this->color[0] = r;
            this->color[1] = g;
            this->color[2] = b;
            this->on = false;
            this->black[0] = 0;
            this->black[1] = 0;
            this->black[2] = 0;
        }

        void turnOn(void) {
            simSetShapeColor(this->handle, NULL, 0, this->color);
            this->on = true;
        }

        void turnOff(void) {
            simSetShapeColor(this->handle, NULL, 0, this->black);
            this->on = false;
        }

        void toggle(void) {
            this->on = !this->on;
            simSetShapeColor(this->handle, NULL, 0, this->on ? this->color : this->black);
        }
};


class Motor {

    private:

        int propHandle;
        int jointHandle;

    public:

        Motor(void) { }

        Motor(int ph, int jh) {
            this->propHandle = ph;
            this->jointHandle = jh;
        }

};

struct Quadcopter
{
    int handle;
    int accelHandle;

    Motor motor1;

    LED  redLED;
    LED  greenLED;
};

static Quadcopter quadcopter;

// simExtHackflight_create -------------------------------------------------------------------------

#define LUA_CREATE_COMMAND "simExtHackflight_create"

// Five handles: quadcopter + four propellers
static const int inArgs_CREATE[]={
    6,
    sim_script_arg_int32,0, // quadcopter handle
    sim_script_arg_int32,0, // accelerometer handle
    sim_script_arg_int32,0, // green LED handle
    sim_script_arg_int32,0, // red LED handle
    sim_script_arg_int32,0, // propeller 1 handle
    sim_script_arg_int32,0, // joint 1 handle
};

void LUA_CREATE_CALLBACK(SScriptCallBack* cb)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(cb->stackID,inArgs_CREATE,inArgs_CREATE[0],LUA_CREATE_COMMAND)) {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        quadcopter.handle         = inData->at(0).int32Data[0];
        quadcopter.accelHandle    = inData->at(1).int32Data[0];

        quadcopter.motor1 = Motor(inData->at(4).int32Data[0], inData->at(5).int32Data[0]);

        quadcopter.greenLED = LED(inData->at(2).int32Data[0], 0, 255, 0);
        quadcopter.redLED   = LED(inData->at(3).int32Data[0], 255, 0, 0);
    }
    D.pushOutData(CScriptFunctionDataItem(true)); // success
    D.writeDataToStack(cb->stackID);
}

// simExtHackflight_destroy --------------------------------------------------------------------

#define LUA_DESTROY_COMMAND "simExtHackflight_destroy"

void LUA_DESTROY_CALLBACK(SScriptCallBack* cb)
{
    CScriptFunctionData D;
    D.pushOutData(CScriptFunctionDataItem(true)); // success
    D.writeDataToStack(cb->stackID);
}


// simExtHackflight_start ------------------------------------------------------------------------

#define LUA_START_COMMAND "simExtHackflight_start"

void LUA_START_CALLBACK(SScriptCallBack* cb)
{
    CScriptFunctionData D;
    cb->waitUntilZero=1; // the effect of this is that when we leave the callback, the Lua script gets control
	   				     // back only when this value turns zero. This allows for "blocking" functions.
    D.pushOutData(CScriptFunctionDataItem(true)); // success
    D.writeDataToStack(cb->stackID);

    setup();
}

// simExtHackflight_stop --------------------------------------------------------------------------------

#define LUA_STOP_COMMAND "simExtHackflight_stop"

void LUA_STOP_CALLBACK(SScriptCallBack* cb)
{
    CScriptFunctionData D;
    D.pushOutData(CScriptFunctionDataItem(true)); // success
    D.writeDataToStack(cb->stackID);
}


VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{ // This is called just once, at the start of V-REP.
    // Dynamically load and bind V-REP functions:
    char curDirAndFile[1024];
    getcwd(curDirAndFile, sizeof(curDirAndFile));

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);

    temp+="/libv_rep.so";

    vrepLib=loadVrepLibrary(temp.c_str());
    if (vrepLib==NULL)
    {
        std::cout << "Error, could not find or correctly load v_rep.dll. Cannot start 'Hackflight' plugin.\n";
        return(0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib)==0)
    {
        std::cout << "Error, could not find all required functions in v_rep.dll. Cannot start 'Hackflight' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }

    // Check the V-REP version:
    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
    if (vrepVer<30200) // if V-REP version is smaller than 3.02.00
    {
        std::cout << "Sorry, V-REP 3.2.0 or higher is required. Cannot start 'Hackflight' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }

    // Register 4 new Lua commands:

    simRegisterScriptCallbackFunction(strConCat(LUA_CREATE_COMMAND,"@",PLUGIN_NAME),
            strConCat("number success=",LUA_CREATE_COMMAND,
                "(number quadcopter, number accel, number greenLED, number redLED, "
                "number prop1, number joint1)"),
            LUA_CREATE_CALLBACK);

    simRegisterScriptCallbackFunction(strConCat(LUA_DESTROY_COMMAND,"@",PLUGIN_NAME),
            strConCat("boolean success=",LUA_DESTROY_COMMAND,"()"),
            LUA_DESTROY_CALLBACK);

    simRegisterScriptCallbackFunction(strConCat(LUA_START_COMMAND,"@",PLUGIN_NAME),
            strConCat("boolean success=", LUA_START_COMMAND,"()"),
            LUA_START_CALLBACK);

    simRegisterScriptCallbackFunction(strConCat(LUA_STOP_COMMAND,"@",PLUGIN_NAME),
            strConCat("boolean success=",LUA_STOP_COMMAND,"()"),LUA_STOP_CALLBACK);

    return(8); // return the version number of this plugin (can be queried with simGetModuleName)
}

VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP
    unloadVrepLibrary(vrepLib); // release the library
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{   
    // This is called quite often. Just watch out for messages/events you want to handle
    // This function should not generate any error messages:

    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

    void* retVal=NULL;

    if (message==sim_message_eventcallback_modulehandle)
    {
        // is the command also meant for Hackflight?
        if ( (customData==NULL)||(std::string("Hackflight").compare((char*)customData)==0) ) 
        {
        }
    }

    loop();

    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings

    return(retVal);
}

// Board implementation --------------------------------------------------------------

#include "../firmware/pwm.hpp"
#include "../firmware/board.hpp"

// V-REP memory model seems to prevent us from making these instance variables of a Board object
static int pwm[8];
static int joyfd;
static uint32_t start_time_usec;

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    // XXX use MPU6050 settings for now
    acc1G = 4096;
    gyroScale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    // Use simulation time to mock up gyro from vehicle's Euler angles
    float curr_time_sec = simGetSimulationTime();
    static float prev_time_sec;
    static float prev_angles[3];
    float angles[3];
    static int count; // ignore startup transient
    simGetObjectOrientation(quadcopter.handle, -1, angles);
    float dt_sec = curr_time_sec - prev_time_sec;
    for (int k=0; k<3; ++k)
        gyroADC[k] = (count++ < 10) ? 0 : (int16_t)(4096 * (angles[k] - prev_angles[k]) / M_PI / dt_sec);
    prev_time_sec = curr_time_sec;
    for (int k=0; k<3; ++k)
        prev_angles[k] = angles[k];

    // Read accelerometer angles from force sensor
    float accelForce[3];
    float accelTorque[3];
    if (simReadForceSensor(quadcopter.accelHandle, accelForce, accelTorque) != -1) {
        for (int k=0; k<3; ++k) {
            accADC[k] = 0;//(int)(4096 * 100 * accelForce[k]);
        }
    }
}

void Board::init(uint32_t & imuLooptimeUsec)
{
    // Initialize "microseconds" timer (10 msec precision);
    start_time_usec = getMicros();

    // Close joystick if open
    if (joyfd > 0)
        close(joyfd);

    // Initialize joystick
    joyfd = open( JOY_DEV , O_RDONLY);
    if(joyfd > 0) 
        fcntl(joyfd, F_SETFL, O_NONBLOCK);

    // Set initial fake PWM values to middle of range
    for (int k=0; k<CONFIG_RC_CHANS; ++k)  {
        pwm[k] = (CONFIG_PWM_MIN + CONFIG_PWM_MAX) / 2;
    }

    // Special treatment for throttle and switch: start them at the bottom
    // of the range.  As soon as they are moved, their actual values will
    // be returned by Board::readPWM().
    pwm[2] = CONFIG_PWM_MIN;
    pwm[4] = CONFIG_PWM_MIN;

    // Fastest rate we can get in V-REP = 10 msec
    imuLooptimeUsec = 5000;
}

void Board::checkReboot(bool pendReboot)
{
}

void Board::delayMilliseconds(uint32_t msec)
{
    usleep(msec*1000);
}

uint32_t Board::getMicros()
{
    return (int)(1e6 * simGetSimulationTime());
}

void Board::ledGreenOff(void)
{
    quadcopter.greenLED.turnOff();
}

void Board::ledGreenOn(void)
{
    quadcopter.greenLED.turnOn();
}

void Board::ledGreenToggle(void)
{
    quadcopter.greenLED.toggle();
}


void Board::ledRedOff(void)
{
    quadcopter.redLED.turnOff();
}

void Board::ledRedOn(void)
{
    quadcopter.redLED.turnOn();
}

void Board::ledRedToggle(void)
{
    quadcopter.redLED.toggle();
}


uint16_t Board::readPWM(uint8_t chan)
{
    if (joyfd > 0) {

        struct js_event js;
        read(joyfd, &js, sizeof(struct js_event));

        if (js.type & ~JS_EVENT_INIT) {
            int fakechan = 0;
            int dir = +1;
            switch (js.number) {
                case 0:
                    fakechan = 3;
                    break;
                case 1:
                    fakechan = 1;
                    break;
                case 2:
                    fakechan = 2;
                    dir = -1;
                    break;
                case 3:
                    fakechan = 4;
                    break;
                case 5:
                    fakechan = 5;
                    break;
            }

            if (fakechan > 0)
                pwm[fakechan-1] = 
                        CONFIG_PWM_MIN + (int)((dir*js.value + 32767)/65534. * 
                                (CONFIG_PWM_MAX-CONFIG_PWM_MIN));
        }
    }

    return pwm[chan];
}

void Board::reboot(void)
{
}

uint8_t Board::serialAvailableBytes(void)
{
    return 0;
}

uint8_t Board::serialReadByte(void)
{
    return 0;
}

void Board::serialWriteByte(uint8_t c)
{
}

void Board::writeMotor(uint8_t index, uint16_t value)
{
    //float force = 0;
    //float torque = 1;
    //simAddForceAndTorque(quadcopter.prop1handle, &force, &torque);
}


