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

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include "wall.hpp"

class ObstacleParser {

    public:

        void parse(string world_file_name)
        {
            ifstream file(world_file_name);

            string line;
            
            while (getline(file, line)) {

                static Wall * _wallptr;

                if (string_contains(line, "Wall {")) {
                    _wallptr = new Wall();
                    _wallptr->rotation_w = 0;
                    _wallptr->rotation_x = 0;
                    _wallptr->rotation_y = 1;
                    _wallptr->rotation_z = 0;

                }

                if (_wallptr) {

                    if (string_contains(line, "translation")) {
                        const auto toks = split_string_by_char(line, ' ');
                        _wallptr->translation_x = stof(toks[1]);
                        _wallptr->translation_y = stof(toks[2]);
                        _wallptr->translation_z = stof(toks[3]);
                    }

                    if (string_contains(line, "rotation")) {
                        const auto toks = split_string_by_char(line, ' ');
                        _wallptr->rotation_x = stof(toks[1]);
                        _wallptr->rotation_x = stof(toks[2]);
                        _wallptr->rotation_y = stof(toks[3]);
                        _wallptr->rotation_z = stof(toks[4]);
                    }

                    if (string_contains(line, "size")) {
                        const auto toks = split_string_by_char(line, ' ');
                        _wallptr->size_x = stof(toks[1]);
                        _wallptr->size_y = stof(toks[2]);
                        _wallptr->size_z = stof(toks[3]);
                    }
                }

                if (string_contains(line, "}")) {
                    _walls.push_back(_wallptr);
                    _wallptr = nullptr;
                }
            }
        }

    private:

        vector<Wall *> _walls;

        static bool string_contains(
                const string str, const string substr) 
        {
            return str.find(substr) < str.length();
        }

        static vector<string> split_string_by_char(
                const string& s, char delimiter)
        {
            vector<string> tokens;
            string token;
            // Create a stringstream object initialized with the input string
            stringstream ss(s); 

            // Use getline to extract tokens from the stream, using the delimiter
            while (getline(ss, token, delimiter)) { 
                if (token.length() > 0) {
                    tokens.push_back(token);
                }
            }
            return tokens;
        }


};

