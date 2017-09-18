/*
Example for testing C output of mspPG

Copyright (C) Rob Jones, Alec Singer, Chris Lavin, Blake Liebling, Simon D. Levy 2015

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include "msppg/msppg.h"

void handle_attitude(short angx, short angy, short heading) {

    printf("%+3d %+3d %+3d\n", angx, angy, heading);
}

int main(int argc, char ** argv) {

    msp_parser_t parser;

    msp_parser_init(&parser);

    msp_message_t message = msp_serialize_ATTITUDE(59, 76, 1);

    msp_set_ATTITUDE_handler(&parser, handle_attitude);

    byte b = msp_message_start(&message); 
    while (msp_message_has_next(&message)) {
        msp_parser_parse(&parser, b);
        b=msp_message_get_next(&message);
    }

    return 0;
}
