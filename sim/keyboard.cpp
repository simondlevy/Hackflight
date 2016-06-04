// Adapted from http://www.cplusplus.com/forum/general/5304/

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

/* http://www.flightsimbooks.com/flightsimhandbook/keyboardcontrols.php
   Left Aileron         Keypad 4    
   Center Ailerons      Keypad 5    
   Right Aileron        Keypad 6    
   Up Elevator*         Keypad 2    
   Down Elevator        Keypad 8    
   Left Rudder          Keypad 0    
   Right Rudder         Keypad Enter
   Throttle Increase*   Keypad 9 
   Throttle Decrease*   Keypad 3 
   */

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
                case 65:
                    printf("8\n");
                    break;
                case 66:
                    printf("2\n");
                    break;
                case 67:
                    printf("6\n");
                    break;
                case 68:
                    printf("4\n");
                    break;
                case 69:
                    printf("5\n");
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
