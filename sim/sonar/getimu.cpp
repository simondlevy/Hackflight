#include "msppg.h"
#include "serial.hpp"

#include <stdio.h>
#include <stdlib.h>


class My_ATTITUDE_Handler : public ATTITUDE_Handler {

    private:

        SerialConnection * serialConnection;

    public:

        My_ATTITUDE_Handler(SerialConnection * s) : ATTITUDE_Handler() {

            this->serialConnection = s;
        }

        void handle_ATTITUDE(short angx, short angy, short heading) {

            printf("%+3d %+3d %+3d\n", angx, angy, heading);

            this->sendAttitudeRequest();
        }

        void sendAttitudeRequest(void) {

            MSP_Message request = MSP_Parser::serialize_ATTITUDE_Request();

            for (byte b=request.start(); request.hasNext(); b=request.getNext())
                this->serialConnection->writeBytes((char *)&b, 1);
        }

};

int main(int argc, char ** argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage:   %s PORT BAUD\n", argv[0]);
        fprintf(stderr, "Example: %s /dev/ttyUSB0 57600\n", argv[0]);
        exit(1);

    }

    SerialConnection serialConnection(argv[1], atoi(argv[2]));

    MSP_Parser parser;

    MSP_Message request = MSP_Parser::serialize_ATTITUDE_Request();

    My_ATTITUDE_Handler handler(&serialConnection);

    parser.set_ATTITUDE_Handler(&handler);

    handler.sendAttitudeRequest();

    while (true)  {
        char c = 0;
        serialConnection.readBytes(&c, 1);
        parser.parse(c);
    }
}
