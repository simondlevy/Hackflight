/* 
   Simple class for wall worlds

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

#include "../sim_datatypes.h"

class Wall {

    public:

        vec3_t translation;
        vec4_t rotation;
        vec3_t size;

        Wall()
        {
            rotation.w = 0;
            rotation.x = 0;
            rotation.y = 1;
            rotation.z = 0;
        }

        void dump()
        {
            printf("Wall: \n");

            printf("  translation: x=%+3.3f y=%+3.3f z=%+3.3f\n",
                    translation.x, translation.y, translation.z);

            printf("  rotation: w=%+3.3f x=%+3.3f y=%+3.3f z=%+3.3f\n",
                    rotation.w, rotation.x, rotation.y, rotation.z);

            printf("  size: x=%3.3f y=%3.3f z=%3.3f\n",
                    size.x, size.y, size.z);

            printf("\n");
        }
};
