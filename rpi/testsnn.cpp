/*
   RaspberryPi Bluetooth server code for Hackflight

   Additional installs required:

     Clone https://github.com/simondlevy/posix-utils to ~/Desktop

     sudo apt install libbluetooth-dev

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

#include <snn_util.hpp>

static const float CLIMBRATE_SCALE  = 500;
static const float CLIMBRATE_OFFSET = 26000;

int main(int argc, char ** argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s NETWORK\n", argv[0]);
        exit(1);
    }

    auto snn = SNN(argv[1], "risp");

    (void)snn;

    return 0;
}
