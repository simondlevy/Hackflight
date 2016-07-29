#include "serial.hpp"
#include "msppg.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

int main(int argc, char ** argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s PORTNAME\n", argv[0]);
        exit(1);
    }

    SerialConnection serialConnection(argv[1], 9600, false);

    unsigned char c = 0;

    while (true) {

        // Every 500 msec, send a pose request 
        static unsigned long usec_start;
        struct timeval tv;
        gettimeofday(&tv,NULL);
        if (tv.tv_usec - usec_start > 500000) {
            MSP_Message poseRequest = MSP_Parser::serialize_SLAM_POSE_Request();
            for (byte b = poseRequest.start(); poseRequest.hasNext(); b = poseRequest.getNext())
                serialConnection.writeBytes((char *)&b, 1);
            usec_start = tv.tv_usec;
            printf("sent message\n");
            char c;
            while (serialConnection.readBytes(&c, 1))
                ;
        }
    }

    serialConnection.closeConnection();
}
