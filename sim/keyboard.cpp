// Adapted from  http://www.cplusplus.com/forum/general/5304/
// Key bindings: http://www.flightsimbooks.com/flightsimhandbook/keyboardcontrols.php

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


int main()
{
    struct termios oldSettings, newSettings;

    tcgetattr( fileno( stdin ), &oldSettings );
    newSettings = oldSettings;
    newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &newSettings );

    while ( 1 )
    {
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
            read( fileno( stdin ), &c, 1 );
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
        else if( res < 0 )
        {
            perror( "select error" );
            break;
        }
    }

    tcsetattr( fileno( stdin ), TCSANOW, &oldSettings );
    return 0;
}
