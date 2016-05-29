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
#include <time.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include "v_repExtHackflight.hpp"
#include "scriptFunctionData.h"
#include "v_repLib.h"

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

#define PLUGIN_NAME "Hackflight"

#define JOY_DEV "/dev/input/js0"

// PID parameters ==================================================================

static const double IMU_PITCH_ROLL_Kp  = .6;//.15;
static const double IMU_PITCH_ROLL_Kd  = .1;//.2;
static const double IMU_PITCH_ROLL_Ki  = 0;

static const double IMU_YAW_Kp 	       = 0;//.05;
static const double IMU_YAW_Kd 	       = 0;//.01;
static const double IMU_YAW_Ki         = 0;

// Flight Forces ====================================================================

static const double ROLL_DEMAND_FACTOR   = 1;
static const double PITCH_DEMAND_FACTOR  = 1;
static const double YAW_DEMAND_FACTOR    = 6;

class PID_Controller {
    
    // General PID control class. 

    private:

        double Kp;
        double Ki;
        double Kd;

        double Eprev;
        double Stdt;
        double t;

    protected:

        PID_Controller(void) { }

        void init(double _Kp, double _Ki, double _Kd) {

            this->Kp = _Kp;
            this->Ki = _Ki;
            this->Kd = _Kd;

            this->Eprev = 0;
            this->Stdt = 0;
            this->t = 0;
        }

        double getCorrection(double target, double actual, double dt=1) {

            double E = target - actual;

            // dE / dt
            double dEdt = this->t > 0 ? (E - this->Eprev) / dt : 0;

            // Integral E / dt 
            this->Stdt += this->t > 0 ? E*dt : 0;

            // Correcting
            double correction = this->Kp*E + this->Ki*this->Stdt + this->Kd*dEdt;

            // Update
            this->t += 1;
            this->Eprev = E;

            return correction;
        }
};

class Demand_PID_Controller: public PID_Controller {
    
    // A class to handle the interaction of demand (joystick, transmitter) and PID control.
    // Control switches from demand to PID when demand falls below a given threshold.

    private:

        double noise_threshold;
        double prevAbsDemand;
        double target;

    public:

        Demand_PID_Controller(void) : PID_Controller() { }
        
        void init(double Kp, double Kd, double Ki=0, double demand_noise_threshold=.01) {

            PID_Controller::init(Kp, Ki, Kd);
            
            // Noise threshold for demand
            this->noise_threshold = demand_noise_threshold;

            this->prevAbsDemand = 1;
            this->target        = 0;
        }

    double getCorrection(double sensorValue, double demandValue, double timestep=1) {
        
        // Returns current PID correction based on sensor value and demand value.
        
        // Assume no correction
        double correction = 0;
        
        // If there is currently no demand
        if (abs(demandValue) < this->noise_threshold) {
        
            // But there was previously a demand
            if (this->prevAbsDemand > this->noise_threshold) {
            
                // Grab the current sensor value as the target
                this->target = sensorValue;
            }
                              
            // With no demand, always need a correction 
            correction = PID_Controller::getCorrection(this->target, sensorValue, timestep);
        }
                    
        // Track previous climb demand, angle    
        this->prevAbsDemand = abs(demandValue);
                                
        return correction;
    }
};

class Stability_PID_Controller: public PID_Controller {

    public:

        // A class to support pitch/roll stability.  K_i parameter and target angle are zero.

        Stability_PID_Controller(void) : PID_Controller() { }

        void init(double Kp, double Kd, double Ki=0) {
            PID_Controller::init(Kp, Ki, Kd);
        }

        double getCorrection(double actualAngle, double timestep=1) {

            // Returns current PID correction based on IMU angle in radians.

            return PID_Controller::getCorrection(0, actualAngle, timestep);
        }
};

class Yaw_PID_Controller : public Demand_PID_Controller {

    // A class for PID control of quadrotor yaw.
    // Special handling is needed for yaw because of numerical instabilities when angle approaches Pi radians
    // (180 degrees).

    public:

        Yaw_PID_Controller(void) : Demand_PID_Controller() { }

        void init(double Kp, double Kd, double Ki, double demand_noise_threshold=.01) {
            Demand_PID_Controller::init(Kp, Kd, Ki, demand_noise_threshold);
        }

        double getCorrection(double yawAngle, double yawDemand, double timestep=1) {

            // Returns current PID correction based on yaw angle in radians value and demand value in interval [-1,+1].

            double correction =  Demand_PID_Controller::getCorrection(-yawAngle, yawDemand, timestep);

            return abs(correction) < 10 ? correction : 0;
        }
};

static Yaw_PID_Controller yaw_IMU_PID;
static Stability_PID_Controller pitch_Stability_PID;
static Stability_PID_Controller roll_Stability_PID;

// Joystick support
static int joyfd;
static double rollDemand;
static double pitchDemand;
static double yawDemand;
static double throttleDemand;

// Timestep for current run
static double timestep;

// Previous angles for integrating gyro
static double anglesPrev[3];

// Library support
static LIBRARY vrepLib;

// Hackflight interface
extern void setup(void);
extern void loop(void);
static uint32_t micros;

// simExtHackflight_start ////////////////////////////////////////////////////////////////////////

#define LUA_START_COMMAND "simExtHackflight_start"

static const int inArgs_START[]={
    1,
    sim_script_arg_double,0 // timestep
};

void LUA_START_CALLBACK(SScriptCallBack* cb)
{
    // Initialize joystick
    joyfd = open( JOY_DEV , O_RDONLY);
    if(joyfd > 0) 
        fcntl(joyfd, F_SETFL, O_NONBLOCK);

    // Special handling for throttle
    throttleDemand = 0;

    // Initialize previous angles
    for (int k=0; k<3; ++k)
        anglesPrev[k] = 0;

    // Initialize PID controllers
    yaw_IMU_PID.init(IMU_YAW_Kp, IMU_YAW_Kd, IMU_YAW_Ki);
    pitch_Stability_PID.init(IMU_PITCH_ROLL_Kp, IMU_PITCH_ROLL_Kd, IMU_PITCH_ROLL_Ki);
    roll_Stability_PID.init(IMU_PITCH_ROLL_Kp, IMU_PITCH_ROLL_Kd, IMU_PITCH_ROLL_Ki);

    // Run Hackflight setup()
    setup();

    // Grab timestep from input stack and return success
    CScriptFunctionData D;
    if (D.readDataFromStack(cb->stackID,inArgs_START,inArgs_START[0],LUA_START_COMMAND)) {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        timestep = inData->at(0).doubleData[0];  
    }
    D.pushOutData(CScriptFunctionDataItem(true)); // success
    D.writeDataToStack(cb->stackID);

}

// simExtHackflight_update ////////////////////////////////////////////////////////////////////////

#define LUA_UPDATE_COMMAND "simExtHackflight_update"

static const int inArgs_UPDATE[]={
    2,
    sim_script_arg_double|sim_script_arg_table,3, // Gyro values
    sim_script_arg_double|sim_script_arg_table,3  // Accelerometer values
};

void LUA_UPDATE_CALLBACK(SScriptCallBack* cb)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(cb->stackID,inArgs_UPDATE,inArgs_UPDATE[0],LUA_UPDATE_COMMAND)) {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();

        // Read gyro, accelerometer
        double angles[3];
        double gyro[3];
        //double accel[3];
        for (int k=0; k<3; ++k) {
            gyro[k]   = inData->at(0).doubleData[k]; 
            //accel[k]  = inData->at(1).doubleData[k]; 
            angles[k] = gyro[k] * timestep + anglesPrev[k]; // Integrate gyro to get angle
            anglesPrev[k] = angles[k];
        }

        double pitchAngle = angles[0];
        double rollAngle  = angles[1];
        double yawAngle   = angles[2]; 

        // Get corrections from PID controllers
        double yawCorrection   = yaw_IMU_PID.getCorrection(yawAngle, yawDemand, timestep);
        double pitchCorrection = pitch_Stability_PID.getCorrection(pitchAngle, timestep);
        double rollCorrection  = roll_Stability_PID.getCorrection(rollAngle, timestep);

        // Baseline thrust is a nonlinear function of climb demand
        double baselineThrust = 4*sqrt(sqrt(throttleDemand)) + 2;

        // Overall thrust is baseline plus throttle demand plus correction from PD controller
        // received from the joystick and the # quadrotor model corrections. A positive 
        // pitch value means, thrust increases for two back propellers and a negative 
        // is opposite; similarly for roll and yaw.  A positive throttle value means thrust 
        // increases for all 4 propellers.
        double psign[4] = {-1, +1, -1, +1}; 
        double rsign[4] = {+1, +1, -1, -1};
        double ysign[4] = {-1, +1, +1, -1};

        // Set thrust for each motor
        for (int i=0; i<4; ++i) {
            double thrust = (baselineThrust + 
                    rsign[i]*rollDemand*ROLL_DEMAND_FACTOR + 
                    psign[i]*pitchDemand*PITCH_DEMAND_FACTOR + 
                    ysign[i]*yawDemand*YAW_DEMAND_FACTOR) * 
                (1 + rsign[i]*rollCorrection + psign[i]*pitchCorrection + ysign[i]*yawCorrection);

            char signame[10];
            sprintf(signame, "thrust%d", i+1);
            simSetFloatSignal(signame, thrust);
        }
    }

    // Increment microsecond count
    micros += 1e6 * timestep;

    // Return success
    D.pushOutData(CScriptFunctionDataItem(true)); 
    D.writeDataToStack(cb->stackID);
}

// simExtHackflight_stop ////////////////////////////////////////////////////////////////////////////////

#define LUA_STOP_COMMAND "simExtHackflight_stop"

void LUA_STOP_CALLBACK(SScriptCallBack* cb)
{
    // Close joystick if open
    if (joyfd > 0)
        close(joyfd);

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

    // Register new Lua commands:
    simRegisterScriptCallbackFunction(strConCat(LUA_START_COMMAND,"@",PLUGIN_NAME), NULL, LUA_START_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_UPDATE_COMMAND,"@",PLUGIN_NAME), NULL, LUA_UPDATE_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_STOP_COMMAND,"@",PLUGIN_NAME), NULL, LUA_STOP_CALLBACK);

    return(8); // return the version number of this plugin (can be queried with simGetModuleName)
}

VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP
    unloadVrepLibrary(vrepLib); // release the library
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{   
    // Read joystick
    if (joyfd > 0) {
        struct js_event js;
        read(joyfd, &js, sizeof(struct js_event));
        if (js.type & ~JS_EVENT_INIT) {
            switch (js.number) {
                case 0:
                    throttleDemand = (js.value + 32767.) / 65534;
                    break;
                case 1:
                    rollDemand = -js.value / 32767.;
                    break;
                case 2:
                    pitchDemand = js.value / 32767.;
                    break;
                case 3:
                    yawDemand = -js.value / 32767.;
            }
        }
    }

    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

    void* retVal=NULL;

    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings

    // Call Hackflight loop() from here for most realistic simulation
    loop();

    return(retVal);
}

// Board implementation ======================================================

#include <board.hpp>

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    acc1G = 4096;
    gyroScale = 1e-6;
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    for (int k=0; k<3; ++k)
        accADC[k] = gyroADC[k] = 0;
}

void Board::init(uint32_t & looptimeMicroseconds)
{
    looptimeMicroseconds = 10000;
}

void Board::checkReboot(bool pendReboot)
{
}

void Board::delayMilliseconds(uint32_t msec)
{
    //usleep(1000*msec);
}

uint32_t Board::getMicros()
{
    return micros; 
}

static bool green;

void Board::ledGreenOff(void)
{
    green = false;
    simSetIntegerSignal("greenLED", 0);
}

void Board::ledGreenOn(void)
{
    green = true;
    simSetIntegerSignal("greenLED", 1);
}

void Board::ledGreenToggle(void)
{
    green = !green;
    simSetIntegerSignal("greenLED", green ? 1 : 0);
}

static bool red;

void Board::ledRedOff(void)
{
    printf("OFF\n");
    red = false;
    simSetIntegerSignal("redLED", 0);
}

void Board::ledRedOn(void)
{
    printf("ON\n");
    red = true;
    simSetIntegerSignal("redLED", 1);
}

void Board::ledRedToggle(void)
{
    printf("TOGGLE\n");
    red = !red;
    simSetIntegerSignal("redLED", red ? 1 : 0);
}

uint16_t Board::readPWM(uint8_t chan)
{
    return 0;
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
}
