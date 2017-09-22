#include "hackflight.hpp"
#include "controller.hpp"

#include <stdio.h>

int main(int argc, char ** argv)
{
    hf::Controller controller;

    controller.begin();

    while (true) {
        controller.update();
        for (int k=0; k<5; ++k) {
            printf("%4d    ", controller.readChannel(k));
        }
        printf("\n");
    }

    controller.halt();

    return 0;
}
