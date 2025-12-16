/* 
   Simple class for wall obstacles

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

class Wall {

    public:

       double translation_x; 
       double translation_y; 
       double translation_z; 

       double rotation_w; 
       double rotation_x; 
       double rotation_y; 
       double rotation_z; 

       double size_x; 
       double size_y; 
       double size_z; 

       void dump()
       {
           printf("Wall: \n");

           printf("  translation: x=%+3.3f y=%+3.3f z=%+3.3f\n",
                   translation_x, translation_y, translation_z);

           printf("  rotation: w=%+3.3f x=%+3.3f y=%+3.3f z=%+3.3f\n",
                   rotation_w, rotation_x, rotation_y, rotation_z);

           printf("  size: x=%3.3f y=%3.3f z=%3.3f\n",
                   size_x, size_y, size_z);

           printf("\n");
       }
};
