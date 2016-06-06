// Adapted from  http://www.cplusplus.com/forum/general/5304/
// Key bindings: http://www.flightsimbooks.com/flightsimhandbook/keyboardcontrols.php

#include "controller.hpp"

KeyboardController::KeyboardController(void) {
}

KeyboardController::~KeyboardController(void) {

    tcsetattr( fileno( stdin ), TCSANOW, &this->oldSettings );
}

void KeyboardController::init(void) {

    struct termios newSettings;

    tcgetattr(fileno( stdin ), &this->oldSettings);
    newSettings = this->oldSettings;
    newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(fileno( stdin ), TCSANOW, &newSettings);
}

void KeyboardController::getDemands(float & pitchDemand, float & rollDemand, float & yawDemand, float & throttleDemand) {

    pitchDemand = 0;
    rollDemand = 0;
    yawDemand = 0;
    throttleDemand = 0;

    fd_set set;
    struct timeval tv;

    tv.tv_sec = 1;
    tv.tv_usec = 0;

    FD_ZERO( &set );
    FD_SET( fileno( stdin ), &set );

    int res = select( fileno( stdin )+1, &set, NULL, NULL, &tv );

    if( res > 0 )
    {
        char c;
        read(fileno( stdin ), &c, 1);
        switch (c) {
            case 10:
                printf("Right rudder\n");
                break;
            case 50:
                printf("Left rudder\n");
                break;
            case 53:
                printf("Throttle increase\n");
                break;
            case 54:
                printf("Throttle decrease\n");
                break;
            case 65:
                printf("Down elevator\n");
                break;
            case 66:
                printf("Up elevator\n");
                break;
            case 67:
                printf("Right aileron\n");
                break;
            case 68:
                printf("Left aileron\n");
                break;
        }
    }
    else if( res < 0 ) {
        perror( "select error" );
    }

}

int main()
{
    KeyboardController kb;

    kb.init();

    while ( 1 )
    {
        float pitchDemand, rollDemand, yawDemand, throttleDemand;

        kb.getDemands(pitchDemand, rollDemand, yawDemand, throttleDemand);
    }

    return 0;
}
