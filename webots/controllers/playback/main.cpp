/* 
   Hackflight simulator playback

   Copyright (C) 2026 Simon D. Levy

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

#include <string>
using namespace std;

int main(int argc, char ** argv) 
{
    (void)argc;

    const char * logfilename =  argv[1];

    FILE * logfp = fopen(logfilename, "r");

    if (!logfp) {
        fprintf(stderr, "Unable to open file %s for input\n", logfilename);
        exit(1);
    }

    while (true) {

        char line[1000] = {};

        if (!fgets(line, sizeof(line), logfp) != NULL) {
            break;
        }

        printf("%s", line);
    }

    return 0;
}
