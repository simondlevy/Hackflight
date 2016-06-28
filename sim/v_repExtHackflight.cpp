/*
   V-REP simulator plugin code for Hackflight

   Copyright (C) Simon D. Levy, Matt Lubas, and Julio Hidalgo Lopez 2016

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

#include "v_repExtHackflight.h"
#include "scriptFunctionData.h"
#include <iostream>
#include "v_repLib.h"

#include <stdint.h>

#include <iostream>
using namespace std;

// WIN32 support
#include <crossplatform.h>

// We currently support these controllers
enum Controller { NONE, TARANIS, SPEKTRUM, EXTREME3D, PS3 };
static Controller controller;

// Stick demands from controller
static int demands[5];

// Keyboard support

static const float KEYBOARD_INC = 10;

static void kbchange(int index, int dir)
{
    demands[index] += (int)(dir*KEYBOARD_INC);

    if (demands[index] > 1000)
        demands[index] = 1000;

    if (demands[index] < -1000)
        demands[index] = -1000;
}


static void kbincrement(int index)
{
    kbchange(index, +1);
}

static void kbdecrement(int index)
{
    kbchange(index, -1);
}

static void kbRespond(char key, char * keys) 
{
	for (int k=0; k<8; ++k)
		if (key == keys[k]) {
			if (k%2)
				kbincrement(k/2);
			else
				kbdecrement(k/2);
        }
}

#ifdef _WIN32 // ===================================================================

#ifdef QT_COMPIL
#include <direct.h>
#else
#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#endif

#include <conio.h>

// Adapted from http://cboard.cprogramming.com/windows-programming/114294-getting-list-usb-devices-listed-system.html
static void controllerInit(void)
{ 
    // Get Number Of Devices
    UINT nDevices = 0;
    GetRawInputDeviceList( NULL, &nDevices, sizeof( RAWINPUTDEVICELIST ) );
 
    // Got Any?
    if(nDevices < 1)
        return;
     
    // Allocate Memory For Device List
    PRAWINPUTDEVICELIST pRawInputDeviceList;
    pRawInputDeviceList = new RAWINPUTDEVICELIST[ sizeof( RAWINPUTDEVICELIST ) * nDevices ];
 
    // Got Memory?
    if( pRawInputDeviceList == NULL ) {
        // Error
        cout << "ERR: Could not allocate memory for Device List.";
        return;
    }
     
    // Fill Device List Buffer
    int nResult;
    nResult = GetRawInputDeviceList( pRawInputDeviceList, &nDevices, sizeof( RAWINPUTDEVICELIST ) );
 
    // Got Device List?
    if( nResult < 0 ) {
        // Clean Up
        delete [] pRawInputDeviceList;
 
        // Error
        cout << "ERR: Could not get device list.";
        return;
    }
 
    // Set Device Info & Buffer Size
    RID_DEVICE_INFO rdiDeviceInfo;
    rdiDeviceInfo.cbSize = sizeof( RID_DEVICE_INFO );
    UINT nBufferSize = rdiDeviceInfo.cbSize;
 
    // Get Device Info
    nResult = GetRawInputDeviceInfo(pRawInputDeviceList[0].hDevice, RIDI_DEVICEINFO, &rdiDeviceInfo, &nBufferSize );
 
    // Got All Buffer?
    if(nResult < 0 ) {
        // Error
        cout << "ERR: Unable to read Device Info." << endl;
        return;
    }
  
    // Some HID
    if (rdiDeviceInfo.dwType == RIM_TYPEHID) {

        switch (rdiDeviceInfo.hid.dwVendorId) {

		case 3727:
			controller = PS3;
			break;

		case 1155:
			controller = TARANIS;
			break;

		case 1783:
			 controller = SPEKTRUM;
			 break;

		case 1133:
			 controller = EXTREME3D; // XXX product ID = 49685
			 break;
		}
            
			// XXX could also use if needed: rdiDeviceInfo.hid.dwProductId
    }
    
    // Clean Up - Free Memory
    delete [] pRawInputDeviceList;
}

// Turns button value into aux-switch demand
static void buttonToAuxDemand(int * values)
{
	int buttons = values[7];

	if (buttons == 1)
		demands[4] = -1000;

	if (buttons == 2)
		demands[4] = 0;

	if (buttons == 4)
		demands[4] = +1000;
}

// Grabs stick demands from script via Windows plugin
static void controllerRead(int * values)
{
    // Handle each controller differently
    switch (controller) {

    case TARANIS:
        demands[0] = values[0];    // roll
        demands[1] = values[1];    // pitch
        demands[2] = values[2];    // yaw
        demands[3] = values[3];    // throttle			
		demands[4] = values[4];    // aux switch
        break;

    case SPEKTRUM:
        demands[0] = values[1];		// roll
        demands[1] = values[2];		// pitch
        demands[2] = values[5];		// yaw
        demands[3] = values[0];		// throttle
        break;

    case EXTREME3D:
        demands[0] =  values[0];	// roll
        demands[1] = -values[1];	// pitch
        demands[2] =  values[5];	// yaw
        demands[3] = -values[6];	// throttle
		buttonToAuxDemand(values);  // aux switch
        break;

    case PS3:
        demands[0] =  values[2];	// roll
        demands[1] = -values[5];	// pitch
        demands[2] =  values[0];	// yaw
        demands[3] = -values[1];	// throttle
		buttonToAuxDemand(values);  // aux switch
		break;

	default:
		 if (_kbhit()) {
            char c = _getch();
			char keys[8] = {52, 54, 50, 56, 48, 13, 51, 57};
			kbRespond(c, keys);
         }
	 }
}

static void controllerClose(void)
{
    // XXX no action needed (?)
}
#endif /* _WIN32 */

// joystick support for Linux
#ifdef __linux // ===================================================================

#define JOY_DEV "/dev/input/js0"

#include <linux/joystick.h>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>

static int joyfd;

static int axismap[5];
static int axisdir[5];

static struct termios oldSettings;

static void controllerInit(void)
{ 
    joyfd = open(JOY_DEV, O_RDONLY);

    for (int k=0; k<5; ++k)
        axisdir[k] = +1;

    if (joyfd > 0) {

        fcntl(joyfd, F_SETFL, O_NONBLOCK);

        char name[128];
        if (ioctl(joyfd, JSIOCGNAME(sizeof(name)), name) < 0)
            printf("Uknown controller\n");

        if (strstr(name, "Taranis")) {
            controller = TARANIS;
            axismap[0] = 0;
            axismap[1] = 1;
            axismap[2] = 2;
            axismap[3] = 3;
            axismap[4] = 4;
        }
        else if (strstr(name, "PPM TO USB Adapter")) {
            controller = SPEKTRUM;
            axismap[0] = 1;
            axismap[1] = 2;
            axismap[2] = 5;
            axismap[3] = 0;
            axismap[4] = 3;
        }
        else if (strstr(name, "MY-POWER CO.")) {
            controller = PS3;
            axismap[0] = 2;
            axismap[1] = 3;
            axismap[2] = 0;
            axismap[3] = 1;
            axisdir[1] = -1;
            axisdir[3] = -1;
        }
        else if (strstr(name, "Extreme 3D")) {
            controller = EXTREME3D;
            axismap[0] = 0;
            axismap[1] = 1;
            axismap[2] = 2;
            axismap[3] = 3;
            axisdir[1] = -1;
            axisdir[3] = -1;
        }
        else {
            printf("Uknown controller: %s\n", name);
        }
    }

    // No joystick detected; use keyboard as fallback
    else {

        struct termios newSettings;

        // Save keyboard settings for restoration later
        tcgetattr(fileno( stdin ), &oldSettings);

        // Create new keyboard settings
        newSettings = oldSettings;
        newSettings.c_lflag &= (~ICANON & ~ECHO);
        tcsetattr(fileno(stdin), TCSANOW, &newSettings);
    }
} 

static void controllerRead(void * ignore)
{
    // Have a joystick; grab its axes
    if (joyfd  > 0) {

        struct js_event js;

        read(joyfd, &js, sizeof(struct js_event));

        // Look at all five axes for R/C transmitters, first four for other controllers
        int maxaxis = (controller == TARANIS || controller == SPEKTRUM) ? 5 : 4;

        // Grab demands from axes
        if (js.type & JS_EVENT_AXIS) 
            for (int k=0; k<maxaxis; ++k)
                if (js.number == axismap[k]) 
                    demands[k] = axisdir[k] * (int)(1000. * js.value / 32767);

        // Grab aux demand from buttons when detected
        if (js.type & JS_EVENT_BUTTON && js.value) {
            switch (js.number) {
                case 0:
                    demands[4] = -1000;
                    break;
                case 1:
                    demands[4] =     0;
                    break;
                case 2:
                    demands[4] = +1000;
            }
        }
    }

    // No joystick; use keyboard
    else {

        fd_set set;
        struct timeval tv;

        tv.tv_sec = 0;
        tv.tv_usec = 100;

        FD_ZERO( &set );
        FD_SET( fileno( stdin ), &set );

        int res = select(fileno(stdin )+1, &set, NULL, NULL, &tv);

        char c = 0;

        if( res > 0 )
        {
            read(fileno( stdin ), &c, 1);
            char keys[8] = {68, 67, 66, 65, 50, 10, 54, 53};
            kbRespond(c, keys);
        }

        else if( res < 0 ) {
            perror( "select error" );
        }
    }
}

static void controllerClose(void)
{
    if (joyfd > 0)
        close(joyfd);

    else // reset keyboard if no joystick
        tcsetattr(fileno(stdin), TCSANOW, &oldSettings);
}
#endif // Linux

// joystick support for OS X
#ifdef __APPLE__  // ===================================================================

#include <SDL.h>

static SDL_Joystick * joystick;

static int axismap[4];
static int axisdir[4];

static void controllerInit(void)
{
    if (SDL_Init(SDL_INIT_JOYSTICK)) {
        printf("Failed to initialize SDL\n");
        return;
    }

    if (!(joystick = SDL_JoystickOpen(0))) {
        printf("Unable to open joystick\n");
        return;
    }

    for (int k=0; k<4; ++k)
        axisdir[k] = +1;

    char name[100];
    strcpy(name, SDL_JoystickNameForIndex(0));

    if (strstr(name, "Taranis")) {
        controller = TARANIS;
        axismap[0] = 0;
        axismap[1] = 1;
        axismap[2] = 2;
        axismap[3] = 3;
    }
    else if (strstr(name, "PPM TO USB Adapter")) {
        controller = SPEKTRUM;
        printf("Spektrum PPM to USB currently not supported on OS X\n");
        joystick = NULL;
    }
    else if (strstr(name, "2In1 USB Joystick")) {
        controller = PS3;
        printf("PS3 currently not supported on OS X\n");
        joystick = NULL;
    }
    else if (strstr(name, "Extreme 3D")) {
        controller = EXTREME3D;
        printf("PS3 currently not supported on OS X\n");
        joystick = NULL;
    }
    else {
        printf("Uknown controller: %s\n", name);
    }
}

static void controllerRead(void * ignore)
{
    if (!joystick)
        return;

    // Poll until we get an event
    SDL_Event event;
    while (SDL_PollEvent(&event))
        ;

    if (event.type == SDL_JOYAXISMOTION) {
        SDL_JoyAxisEvent js = event.jaxis;
        for (int k=0; k<4; ++k)
            if (js.axis == axismap[k]) 
                demands[k] = axisdir[k] * (int)(1000. * js.value / 32767);
    }
}

static void controllerClose(void)
{
    if (joystick)
        SDL_JoystickClose(joystick);
}

#endif // OS X

#if defined (__linux) || defined (__APPLE__)
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define WIN_AFX_MANAGE_STATE
#endif /* __linux || __APPLE__ */

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

#define PLUGIN_NAME "Hackflight"

LIBRARY vrepLib;

// Hackflight interface
extern void setup(void);
extern void loop(void);
static uint32_t micros;

// Launch support
static bool ready;

// needed for spring-mounted throttle stick
static int throttleDemand;
#define PS3_THROTTLE_INC .005                

// IMU support
static double accel[3];
static double gyro[3];

// Barometer support
static int baroPressure;

// Sonar support
static int sonarAGL;

// Motor support
static float thrusts[4];

// 100 Hz timestep, used for simulating microsend timer
static double timestep = .01;


// --------------------------------------------------------------------------------------
// simExtHackflight_start
// --------------------------------------------------------------------------------------
#define LUA_START_COMMAND "simExtHackflight_start"

void LUA_START_CALLBACK(SScriptCallBack* cb)
{

    // Run Hackflight setup()
    setup();

    // Need this for throttle on keyboard and PS3
    throttleDemand = -1000;

    // For safety, all controllers start at minimum throttle, aux switch off
    demands[3] = -1000;
	demands[4] = -1000;

    // Each input device has its own axis and button mappings
    controllerInit();

    // Now we're ready
    ready = true;

    // Return success to V-REP
    CScriptFunctionData D;
    D.pushOutData(CScriptFunctionDataItem(true));
    D.writeDataToStack(cb->stackID);
}

// --------------------------------------------------------------------------------------
// simExtHackflight_update
// --------------------------------------------------------------------------------------

#define LUA_UPDATE_COMMAND "simExtHackflight_update"

static const int inArgs_UPDATE[]={
    5,
    sim_script_arg_int32|sim_script_arg_table,8,  // Controller values
    sim_script_arg_double|sim_script_arg_table,3, // Gyro values
    sim_script_arg_double|sim_script_arg_table,3, // Accelerometer values
    sim_script_arg_int32,0,                       // Barometric pressure
    sim_script_arg_int32,0                        // Sonar distance
};

void LUA_UPDATE_CALLBACK(SScriptCallBack* cb)
{
    CScriptFunctionData D;

    if (D.readDataFromStack(cb->stackID,inArgs_UPDATE,inArgs_UPDATE[0],LUA_UPDATE_COMMAND)) {

        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();

        // Controller values from script will be used in Windows only
        controllerRead(&inData->at(0).int32Data[0]);

        // PS3 spring-mounted throttle requires special handling
        if (controller == PS3) {
            throttleDemand += (int)(demands[3] * PS3_THROTTLE_INC);     
            if (throttleDemand < -1000)
                throttleDemand = -1000;
            if (throttleDemand > 1000)
                throttleDemand = 1000;
        }
        else {
            throttleDemand = demands[3];
        }

        // Read gyro, accelerometer
        for (int k=0; k<3; ++k) {
            gyro[k]   = inData->at(1).doubleData[k]; 
            accel[k]  = inData->at(2).doubleData[k]; 
        }

        // Read barometer
        baroPressure = inData->at(3).int32Data[0];

        // Read sonar
        sonarAGL = inData->at(4).int32Data[0];

        printf("********** %d\n", sonarAGL);

        // Set thrust for each motor
        for (int i=0; i<4; ++i) {
            char signame[10];
            SPRINTF(signame, "thrust%d", i+1);
            simSetFloatSignal(signame, thrusts[i]);
        }
    }

    // Increment microsecond count
    micros += (uint32_t)(1e6 * timestep);

    // Return success to V-REP
    D.pushOutData(CScriptFunctionDataItem(true)); 
    D.writeDataToStack(cb->stackID);
}


// --------------------------------------------------------------------------------------
// simExtHackflight_stop
// --------------------------------------------------------------------------------------
#define LUA_STOP_COMMAND "simExtHackflight_stop"


void LUA_STOP_CALLBACK(SScriptCallBack* cb)
{
    controllerClose();

    CScriptFunctionData D;
    D.pushOutData(CScriptFunctionDataItem(true));
    D.writeDataToStack(cb->stackID);
}
// --------------------------------------------------------------------------------------


VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{ // This is called just once, at the start of V-REP.
    // Dynamically load and bind V-REP functions:
    char curDirAndFile[1024];
#ifdef _WIN32
#ifdef QT_COMPIL
    _getcwd(curDirAndFile, sizeof(curDirAndFile));
#else
    GetModuleFileName(NULL,curDirAndFile,1023);
    PathRemoveFileSpec(curDirAndFile);
#endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);

#ifdef _WIN32
    temp+="\\v_rep.dll";
#elif defined (__linux)
    temp+="/libv_rep.so";
#elif defined (__APPLE__)
    temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */

    vrepLib=loadVrepLibrary(temp.c_str());
    if (vrepLib==NULL)
    {
        std::cout << "Error, could not find or correctly load v_rep.dll. Cannot start 'Hackflight' plugin.\n";
        return(0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib)==0)
    {
        std::cout << "Error, could not find all required functions in v_rep plugin. Cannot start 'Hackflight' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }

    // Check the V-REP version:
    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
    if (vrepVer<30200) // if V-REP version is smaller than 3.02.00
    {
        std::cout << "Sorry, your V-REP copy is somewhat old, V-REP 3.2.0 or higher is required. Cannot start 'Hackflight' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }

    // Register new Lua commands:
    simRegisterScriptCallbackFunction(strConCat(LUA_START_COMMAND,"@",PLUGIN_NAME),strConCat("boolean result=",LUA_START_COMMAND,"(number HackflightHandle,number duration,boolean returnDirectly=false)"),LUA_START_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_UPDATE_COMMAND,"@",PLUGIN_NAME), NULL, LUA_UPDATE_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_STOP_COMMAND,"@",PLUGIN_NAME),strConCat("boolean result=",LUA_STOP_COMMAND,"(number HackflightHandle)"),LUA_STOP_CALLBACK);

    return 8; // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP
    unloadVrepLibrary(vrepLib); // release the library
}

#ifdef CONTROLLER_KEYBOARD
static void change(int index, int dir)
{
    demands[index] += dir*KEYBOARD_INC;

    if (demands[index] > 1000)
        demands[index] = 1000;

    if (demands[index] < -1000)
        demands[index] = -1000;
}

static void increment(int index) 
{
    change(index, +1);
}

static void kbdecrement(int index) 
{
    change(index, -1);
}
#endif

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
    // Don't do anything till start() has been called
    if (!ready)
        return NULL;

    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings

    // Call Hackflight loop() from here for most realistic simulation
    loop();

    return NULL;
}

// Board implementation ======================================================

#include <board.hpp>
#include <pwm.hpp>

class LED {

    private:

        char signame[10];
        bool on;

        void set(bool status)
        {
            this->on = status;
            simSetIntegerSignal(this->signame, this->on ? 1 : 0);
        }

    public:

        LED(void) { }

        void init(const char * _signame)
        {
            STRCPY(this->signame, _signame);
            this->on = false;
        }

        void turnOff(void) 
        {
            this->set(false);
        }

        void turnOn(void) 
        {
            this->set(true);
        }

        void toggle(void) 
        {
            this->set(!this->on);
        }
};

static LED greenLED, redLED;

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    // Mimic MPU6050
    acc1G = 4096;
    gyroScale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    // Convert from radians to tenths of a degree

    for (int k=0; k<3; ++k) {
        accADC[k]  = (int16_t)(400000 * accel[k]);
    }

    gyroADC[1] = -(int16_t)(1000 * gyro[0]);
    gyroADC[0] = -(int16_t)(1000 * gyro[1]);
    gyroADC[2] = -(int16_t)(1000 * gyro[2]);
}

void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec, bool & initiallyArmed)
{
    looptimeMicroseconds = 10000;
    calibratingGyroMsec = 100;  // long enough to see but not to annoy

    greenLED.init("greenLED");
    redLED.init("redLED");

    initiallyArmed = true;
}

bool Board::baroInit(void)
{
    return true;
}

void Board::baroUpdate(void)
{
}

int32_t Board::baroGetPressure(void)
{
    return baroPressure;
}

void Board::checkReboot(bool pendReboot)
{
}

void Board::delayMilliseconds(uint32_t msec)
{
}

uint32_t Board::getMicros()
{
    return micros; 
}

void Board::ledGreenOff(void)
{
    greenLED.turnOff();
}

void Board::ledGreenOn(void)
{
    greenLED.turnOn();
}

void Board::ledGreenToggle(void)
{
    greenLED.toggle();
}

void Board::ledRedOff(void)
{
    redLED.turnOff();
}

void Board::ledRedOn(void)
{
    redLED.turnOn();
}

void Board::ledRedToggle(void)
{
    redLED.toggle();
}

uint16_t Board::readPWM(uint8_t chan)
{
    //return CONFIG_PWM_MIN;

    int demand = (chan == 3) ? throttleDemand : demands[chan];

    // V-REP sends joystick demands in [-1000,+1000]
    int pwm =  (int)(CONFIG_PWM_MIN + (demand + 1000) / 2000. * (CONFIG_PWM_MAX - CONFIG_PWM_MIN));
	
    //if (chan < 5) printf("%d: %d%s", chan, pwm, chan == 4 ? "\n" : "    ");

    return pwm;
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
    thrusts[index] = 4 * ((float)value - CONFIG_PWM_MIN) / (CONFIG_PWM_MAX - CONFIG_PWM_MIN) + 2;
}
