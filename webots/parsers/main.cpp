/* 
   Webots world-parsing tester

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

#include "world_parser.hpp"
#include "robot_parser.hpp"

int main(int argc, char ** argv) 
{
    (void)argc;

    const std::string world =  argv[1];
    static WorldParser _worldParser;
    _worldParser.parse(world);
    _worldParser.report();

    const std::string robot =  argv[2];
    static RobotParser _robotParser;
    _robotParser.parse(robot);
    _robotParser.report();

     return 0;
}
