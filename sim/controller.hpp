#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class Controller {

    protected:

        Controller(void) { }

    public :

        virtual void getDemands(float & pitchDemand, float & rollDemand, float & yawDemand, float & throttleDemand) = 0;

};

class AxialController : Controller {
};

class TaranisController : public AxialController {

    void getDemands(float & pitchDemand, float & rollDemand, float & yawDemand, float & throttleDemand);
};

class PS3Controller : public AxialController {

    void getDemands(float & pitchDemand, float & rollDemand, float & yawDemand, float & throttleDemand);
};

class KeyboardController : public Controller {

    private:

        struct termios oldSettings;

    public:

        KeyboardController(void);

        ~KeyboardController(void);

        void init(void);

        void getDemands(float & pitchDemand, float & rollDemand, float & yawDemand, float & throttleDemand);
};

