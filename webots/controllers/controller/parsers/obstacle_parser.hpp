/* 
   Simple VRML parser for Webots proto / proto files

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

class ObstacleParser {

    public:

        void parse(string proto_file_name)
        {
            ifstream file(proto_file_name);

            string line;

            while (getline(file, line)) {

                static Wall * _wallptr;

                if (ParserUtils::string_contains(line, "Wall {")) {
                    _wallptr = new Wall();
                    _wallptr->rotation_w = 0;
                    _wallptr->rotation_x = 0;
                    _wallptr->rotation_y = 1;
                    _wallptr->rotation_z = 0;

                }

                if (_wallptr) {

                    if (ParserUtils::string_contains(line, "translation")) {
                        const auto toks = ParserUtils::split_string(line, ' ');
                        _wallptr->translation_x = stof(toks[1]);
                        _wallptr->translation_y = stof(toks[2]);
                        _wallptr->translation_z = stof(toks[3]);
                    }

                    if (ParserUtils::string_contains(line, "rotation")) {
                        const auto toks = ParserUtils::split_string(line, ' ');
                        _wallptr->rotation_x = stof(toks[1]);
                        _wallptr->rotation_x = stof(toks[2]);
                        _wallptr->rotation_y = stof(toks[3]);
                        _wallptr->rotation_z = stof(toks[4]);
                    }

                    if (ParserUtils::string_contains(line, "size")) {
                        const auto toks = ParserUtils::split_string(line, ' ');
                        _wallptr->size_x = stof(toks[1]);
                        _wallptr->size_y = stof(toks[2]);
                        _wallptr->size_z = stof(toks[3]);
                    }

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

