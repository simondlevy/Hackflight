/* 
   Simple VRML parser for Webots .wbt world files

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
#include "obstacles/wall.hpp"

class WorldParser {

    public:

        void parse(const string world_file_name)
        {
            ifstream file(world_file_name);

            if (file.is_open()) {

                string line;

                Wall * _wall = nullptr;

                while (getline(file, line)) {

                    if (ParserUtils::string_contains(line, "Wall {")) {
                        _wall = new Wall();
                    }

                    if (_wall) {

                        ParserUtils::try_parse_vec3(line, "translation",
                                _wall->translation);
                        ParserUtils::try_parse_vec4(line, "rotation",
                                _wall->rotation);
                        ParserUtils::try_parse_vec3(line, "size",
                                _wall->size);

                        if (ParserUtils::string_contains(line, "}")) {
                            _walls.push_back(_wall);
                            _wall = nullptr;
                        }
                    }
                }
            }

            else {
                fprintf(stderr, "Unable to open file %s for input\n",
                        world_file_name.c_str());
            }
        }

        void report()
        {
            for (auto _wall : _walls) {
                _wall->dump();
            }
        }

    private:

        vector<Wall *> _walls;
};

