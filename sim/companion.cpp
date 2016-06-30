/*
   companion.cpp : Companion-board class implementation

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "companion.hpp"

#ifdef __linux
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#endif

CompanionBoard::CompanionBoard(void)
{
}

void CompanionBoard::start(void)
{
    this->pid = 0;

#ifdef __linux
    char script[200];
    sprintf(script, "%s/hackflight_companion.py", VREP_DIR);
    char *argv[2] = {(char *)script, NULL};

    this->pid = fork();

    if (this->pid == 0) {
        execvp(script, argv);
    }
#endif
}
        
void CompanionBoard::update(char * imageBytes, int imageWidth, int imageHeight)
{
}

void CompanionBoard::halt(void)
{
#ifdef __linux
    if (this->pid)
        kill(this->pid, SIGKILL);
#endif
}
