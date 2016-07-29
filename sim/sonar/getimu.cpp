#include "msppg.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char ** argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage:   %s PORT BAUD\n", argv[0]);
        fprintf(stderr, "Example: %s /dev/ttyUSB0 57600\n", argv[0]);
        exit(1);

    }
}

/*
BAUD = 57600

from msppg import MSP_Parser as Parser, serialize_ATTITUDE_Request
import serial

from sys import argv

if len(argv) < 2:

    print('Usage: python3 %s PORT' % argv[0])
    print('Example: python3 %s /dev/ttyUSB0' % argv[0])
    exit(1)

parser = Parser()
request = serialize_ATTITUDE_Request()
port = serial.Serial(argv[1], BAUD)

def handler(pitch, roll, yaw):

    print(pitch, roll, yaw)
    port.write(request)

parser.set_ATTITUDE_Handler(handler)

port.write(request)

while True:

    try:

        parser.parse(port.read(1))

    except KeyboardInterrupt:

        break
*/
