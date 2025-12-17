/* 
   Simple VRML parser for Webots .proto files

   Copyright (C) 2025 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <stdio.h>

#include "utils.hpp"

class RobotParser {

    public:

        void parse(const string robot_file_name)
        {
            ifstream file(robot_file_name);

            if (file.is_open()) {

                string line;

                while (getline(file, line)) {

                    printf("%s\n", line.c_str());
                }
            }

            else {
                fprintf(stderr, "Unable to open file %s for input\n",
                       robot_file_name.c_str());
            }
        }

};

