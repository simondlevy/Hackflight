/*
   V-REP simulator plugin code 3D SLAM visualization

   Copyright (C) Simon D. Levy, Matt Lubas, and Alfredo Rwagaju 2016

   This file is part of 3DSLAM.

   3DSLAM is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   3DSLAM is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with 3DSLAM.  If not, see <http://www.gnu.org/licenses/>.
*/

static const int PORT = 20000;

#include "v_repExt.h"
#include "scriptFunctionData.h"
#include "v_repLib.h"

#include <stdio.h>
#include <netdb.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#include <string.h>

#include <iostream>
using namespace std;

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

#define PLUGIN_NAME  "3DSLAM"

LIBRARY vrepLib;

// http://web.eecs.utk.edu/~plank/plank/classes/cs360/360/notes/Sockets/sockettome.c
static int serve_socket(int port)
{
    int s;
    struct sockaddr_in sn;
    struct hostent *he;

    if (!(he = gethostbyname("localhost"))) {
        puts("can't gethostname");
        exit(1);
    }

    memset((char*)&sn, 0, sizeof(sn));
    sn.sin_family = AF_INET;
    sn.sin_port = htons((short)port);
    sn.sin_addr.s_addr = htonl(INADDR_ANY);

    if ((s = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket()");
        exit(1);
    }

    if (bind(s, (struct sockaddr *)&sn, sizeof(sn)) == -1) {
        perror("bind()");
        exit(1);
    }

    return s;
}

static int accept_connection(int s)
{
    int x;
    struct sockaddr_in sn;

    if(listen(s, 1) == -1) {
        perror("listen()");
        exit(1);
    }

    bzero((char *)&sn, sizeof(sn));
    
    if((x = accept(s, (struct sockaddr *)NULL, NULL)) == -1) {
        perror("accept()");
        exit(1);
    }
    return x;
}

static int read_from_socket(int clientfd, char * buf, int n)
// http://developerweb.net/viewtopic.php?id=2933
{
    fd_set readset;
    int result = 0;

    do {
        FD_ZERO(&readset);
        FD_SET(clientfd, &readset);
        result = select(clientfd + 1, &readset, NULL, NULL, NULL);
    } while (result == -1 && errno == EINTR);

    if (result > 0) {

        if (FD_ISSET(clientfd, &readset)) {

            // The clientfd has data available to be read 
            result = recv(clientfd, buf, n, 0);

            if (result == 0) {
                // This means the other side closed the socket
                close(clientfd);
            }

            else {
                return n; // Success!
            }
        }
    }

    return 0;
}

static int sockfd, clientfd;

// --------------------------------------------------------------------------------------
// simExt3DSLAM_start
// --------------------------------------------------------------------------------------
#define LUA_START_COMMAND  "simExt3DSLAM_start"

void LUA_START_CALLBACK(SScriptCallBack* cb)
{
    CScriptFunctionData D;

    // Return success to V-REP
    D.pushOutData(CScriptFunctionDataItem(true));
    D.writeDataToStack(cb->stackID);

    sockfd = serve_socket(PORT);
    printf("Waiting for client ...");
    fflush(stdout);
    clientfd = accept_connection(sockfd);
    printf("\nClient connected\n");
}

// --------------------------------------------------------------------------------------
// simExt3DSLAM_update
// --------------------------------------------------------------------------------------

#define LUA_UPDATE_COMMAND "simExt3DSLAM_update"

void LUA_UPDATE_CALLBACK(SScriptCallBack* cb)
{
    char c;
    if (read_from_socket(clientfd, &c, 1))
        printf("%c", c);
/*

   cube = simCreatePureShape(0,          -- cube
   16,         -- static
   {CUBESIZE, CUBESIZE, CUBESIZE},
   0)          -- mass

   simSetObjectPosition(cube, -1, {x,y,z}) -- -1 = absolute position 

*/

    CScriptFunctionData D;

    // Return success to V-REP
    D.pushOutData(CScriptFunctionDataItem(true)); 
    D.writeDataToStack(cb->stackID);

} // LUA_UPDATE_COMMAND


// --------------------------------------------------------------------------------------
// simExt3DSLAM_stop
// --------------------------------------------------------------------------------------
#define LUA_STOP_COMMAND "simExt3DSLAM_stop"


void LUA_STOP_CALLBACK(SScriptCallBack* cb)
{
    CScriptFunctionData D;
    D.pushOutData(CScriptFunctionDataItem(true));
    D.writeDataToStack(cb->stackID);

    close(clientfd);
    close(sockfd);
}

// --------------------------------------------------------------------------------------



VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{ 
    char curDirAndFile[1024];

    getcwd(curDirAndFile, sizeof(curDirAndFile));

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);

    temp+="/libv_rep.so";

    vrepLib=loadVrepLibrary(temp.c_str());
    if (vrepLib==NULL)
    {
        std::cout << "Error, could not find or correctly load v_rep.dll. Cannot start '3DSLAM' plugin.\n";
        return(0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib)==0)
    {
        std::cout << "Error, could not find all required functions in v_rep plugin. Cannot start '3DSLAM' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }

    // Check the V-REP version:
    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
    if (vrepVer<30200) // if V-REP version is smaller than 3.02.00
    {
        std::cout << "Sorry, your V-REP copy is somewhat old, V-REP 3.2.0 or higher is required. Cannot start '3DSLAM' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }

    // Register new Lua commands:
    simRegisterScriptCallbackFunction(strConCat(LUA_START_COMMAND,"@",PLUGIN_NAME),
            strConCat("boolean result=",LUA_START_COMMAND,
                "(number 3DSLAMHandle,number duration,boolean returnDirectly=false)"),LUA_START_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_UPDATE_COMMAND,"@",PLUGIN_NAME), NULL, LUA_UPDATE_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_STOP_COMMAND,"@",PLUGIN_NAME),
            strConCat("boolean result=",LUA_STOP_COMMAND,"(number 3DSLAMHandle)"),LUA_STOP_CALLBACK);

    return 8; // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP
    unloadVrepLibrary(vrepLib); // release the library
}

VREP_DLLEXPORT void* v_repMessage(int message, int * auxiliaryData, void * customData, int * replyData)
{
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings


    return NULL;
}
