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
                        printf("translation = ");
                        const auto toks = split_string_by_char(line, ' ');
                        for (int k=1; k<4; ++k) {
                            printf(" |%f| ", stof(toks[k]));
                        }
                        printf("\n");
                    }

                    if (string_contains(line, "size")) {
                        printf("size = ");
                        const auto toks = split_string_by_char(line, ' ');
                        for (int k=1; k<4; ++k) {
                            printf(" |%f| ", stof(toks[k]));
                        }
                        printf("\n");
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

