#include "hackflight.hpp"
#include "controller.hpp"

#include <stdio.h>

int main(int argc, char ** argv)
{
    hf::Controller controller;

    controller.begin();

    while (true) {
        controller.update();
        printf("%4d\n", controller.readChannel(0));
    }

    controller.halt();

    return 0;
}
