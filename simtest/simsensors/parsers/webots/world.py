'''
Simple VRML parser for Webots .wbt world files

Python port of the C++ WorldParser from simsensors.

Copyright (C) 2025 Simon D. Levy

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, in version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
'''


def parse(world_file_name):
    '''
    Parse a Webots .wbt world file and return a dict with a 'walls' list.
    Each wall has 'translation' (3-list), 'rotation' (4-list), 'size' (3-list),
    and 'name' (string).
    '''

    walls = []
    current_wall = None

    with open(world_file_name) as f:
        for line in f:
            line = line.strip()

            if 'Wall {' in line:
                current_wall = {
                    'translation': [0.0, 0.0, 0.0],
                    'rotation': [0.0, 0.0, 1.0, 0.0],
                    'size': [0.0, 0.0, 0.0],
                    'name': '',
                }

            if current_wall is not None:
                _try_parse_vec3(line, 'translation', current_wall)
                _try_parse_rotation(line, 'rotation', current_wall)
                _try_parse_vec3(line, 'size', current_wall)
                _try_parse_name(line, current_wall)

                if '}' in line:
                    walls.append(current_wall)
                    current_wall = None

    return {'walls': walls}


def _try_parse_vec3(line, field_name, wall):
    if field_name in line:
        toks = line.split()
        if len(toks) >= 4:
            wall[field_name] = [float(toks[1]), float(toks[2]), float(toks[3])]


def _try_parse_rotation(line, field_name, wall):
    if field_name in line:
        toks = line.split()
        if len(toks) >= 5:
            wall[field_name] = [
                float(toks[1]), float(toks[2]),
                float(toks[3]), float(toks[4]),
            ]


def _try_parse_name(line, wall):
    if 'name' in line:
        toks = line.split()
        if len(toks) >= 2:
            # Remove surrounding quotes
            wall['name'] = toks[1].strip('"')
