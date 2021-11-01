/*
 * Test code for joystick on Linux
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#include <Joystick.h>
#include <stdio.h>

int main(int argc, char ** argv)
{
    Joystick js;
    float axes[8] = {0};

    while (true) {

        if (!js.poll(axes)) {

            printf("thr:%+f rol:%+f pit:%+f yaw:%+f aux:%+f\n", axes[0], axes[1], axes[2], axes[3], axes[4]);
        }
    }

    return 0;
}


