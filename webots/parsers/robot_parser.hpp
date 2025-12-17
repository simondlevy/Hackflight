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
#include "sensors/rangefinder.hpp"

class RobotParser {

    public:

        void parse(const string robot_file_name)
        {
            ifstream file(robot_file_name);

            if (file.is_open()) {

                string line;

                Rangefinder * _rangefinder = nullptr;

                while (getline(file, line)) {

                    if (ParserUtils::string_contains(line, "RangeFinder {")) {
                        _rangefinder = new Rangefinder();
                    }

                    if (_rangefinder) {

                        printf("rangefinder: %s\n", line.c_str());

                        if (ParserUtils::string_contains(line, "}")) {
                            _rangefinders.push_back(_rangefinder);
                            _rangefinder = nullptr;
                        }
                    }
                }
            }

            else {
                fprintf(stderr, "Unable to open file %s for input\n",
                       robot_file_name.c_str());
            }
        }

    private:

        vector<Rangefinder *> _rangefinders;
};

