#include <stdio.h>
#include "controller.hpp"

int main(int argc, char ** argv)
{

#ifdef PS3
    PS3Controller controller;
#endif

#ifdef SPEKTRUM
    SpektrumController controller;
#endif

#ifdef TARANIS
    TaranisController controller;
#endif

#ifdef KEYBOARD
    KeyboardController controller;
#endif

    controller.init();

    for (int k=0; ; k++) {

        float rollDemand, pitchDemand, yawDemand, throttleDemand, auxDemand;

        controller.getDemands(rollDemand, pitchDemand, yawDemand, throttleDemand, auxDemand);

        printf("%d r: %+3.3f  | p: %+3.3f | y: %+3.3f  | t: %3.3f | a: %+3.3f\n", 
                k, rollDemand, pitchDemand, yawDemand, throttleDemand, auxDemand);
    }

    return 0;
}
