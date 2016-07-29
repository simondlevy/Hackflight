#include <stdio.h>
#include <stdlib.h>

#include "serial.hpp"

int main(int argc, char ** argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage:   %s PORTNAME\n", argv[0]);
        fprintf(stderr, "Example: %s /dev/ttyUSB0\n", argv[0]);
        exit(1);
    }

    SerialConnection s(argv[1]);

    //while (true)
    //    printf("avail: %d\n", s.bytesAvailable());

    s.closeConnection();

    return 0;
}
