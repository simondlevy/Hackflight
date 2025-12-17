/* 
   Simple rangefinder class

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

class Rangefinder {

    public:

        double field_of_view_radians;
        int width;
        int height;
        double min_distance_m;
        double max_distance_m;

        void dump()
        {
            printf("Rangefinder: \n");

            printf("  fov: %3.3f rad\n", field_of_view_radians);
            printf("  width: %d\n", width);
            printf("  height: %d\n", height);
            printf("  min range: %3.3f m\n", min_distance_m);
            printf("  max range: %3.3f m\n", max_distance_m);

            printf("\n");
        }
};

