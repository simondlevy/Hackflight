#include <stdio.h>
#include "controller.hpp"

int main(int argc, char ** argv)
{

#ifdef PS3
    PS3Controller controller;
#endif

#ifdef TARANIS
    TaranisController controller;
#endif

#ifdef KEYBOARD
    KeyboardController controller;
#endif

    controller.init();

    while (1) {

        float pitchDemand, rollDemand, yawDemand, throttleDemand;

        controller.getDemands(pitchDemand, rollDemand, yawDemand, throttleDemand);

        printf("p: %+3.3f  | r: %+3.3f | y: %+3.3f  | t: %3.3f\n", 
                pitchDemand, rollDemand, yawDemand, throttleDemand);
    }

    return 0;
}
