/* 
   Simple VRML parser for Webots world / proto files

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

// C++
#include <string>
#include <iostream>
#include <fstream>
using namespace std;

class WorldParser {

    public:

        static void parse(string world_file_name)
        {
            ifstream file(world_file_name);

            string line;
            
            while (getline(file, line)) {

                static bool _in_wall;

                if (string_contains(line, "Wall {")) {
                    _in_wall = true;
                }
                if (_in_wall) {
                    if (string_contains(line, "translation")) {
                        cout << line << endl;
                    }
                    if (string_contains(line, "size")) {
                        cout << line << endl;
                    }
                }
                if (string_contains(line, "}")) {
                    _in_wall = false;
                }
             }
        }

    private:

        static bool string_contains(
                const string str, const string substr) 
        {
            return str.find(substr) < str.length();
        }

};

