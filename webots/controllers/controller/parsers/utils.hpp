/* 
   Simple VRML parsing utilities

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
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include "sim_datatypes.h"

class ParserUtils {

    public:

        static void try_parse_vec3(const string line, const string field_name,
                vec3_t & vec) 
        {
            if (string_contains(line, field_name)) {
                const auto toks = split_string(line, ' ');
                vec.x = stof(toks[1]);
                vec.y = stof(toks[2]);
                vec.z = stof(toks[3]);
            }
        }

        static void try_parse_vec4(const string line, const string field_name,
                vec4_t & vec) 
        {
            if (string_contains(line, field_name)) {
                const auto toks = split_string(line, ' ');
                vec.w = stof(toks[1]);
                vec.x = stof(toks[2]);
                vec.y = stof(toks[3]);
                vec.z = stof(toks[4]);
            }
        }

        static bool string_contains(
                const string str, const string substr) 
        {
            return str.find(substr) < str.length();
        }

    private:

        static vector<string> split_string(
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

