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

#include "utils.hpp"
#include "obstacles/wall.hpp"

class WorldParser {

    public:

        void parse(string proto_file_name)
        {
            ifstream file(proto_file_name);

            string line;

            while (getline(file, line)) {

                static Wall * _wallptr;

                if (ParserUtils::string_contains(line, "Wall {")) {
                    _wallptr = new Wall();
                }

                if (_wallptr) {

                    ParserUtils::try_parse_vec3(line, "translation",
                            _wallptr->translation);

                    ParserUtils::try_parse_vec4(line, "rotation",
                            _wallptr->rotation);

                    ParserUtils::try_parse_vec3(line, "size",
                            _wallptr->size);

                    if (ParserUtils::string_contains(line, "}")) {
                        _walls.push_back(_wallptr);
                        _wallptr = nullptr;
                    }
                }
            }
        }

        void report()
        {
            for (auto _wallptr : _walls) {
                _wallptr->dump();
            }
        }

    private:

        vector<Wall *> _walls;
};

