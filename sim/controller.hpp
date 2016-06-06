class Controller {

    protected:

        Controller(void) { }

    public :

        virtual void read(float & pitchDemand, float & rollDemand, float & yawDemand, float & throttleDemand) = 0;

};

class AxialController : Controller {
};

class TaranisController : public AxialController {

    void read(float & pitchDemand, float & rollDemand, float & yawDemand, float & throttleDemand);
};

class PS3Controller : public AxialController {

    void read(float & pitchDemand, float & rollDemand, float & yawDemand, float & throttleDemand);
};

class KeyboardController : public Controller {

    KeyboardController(void);

    void init(void);

    void read(float & pitchDemand, float & rollDemand, float & yawDemand, float & throttleDemand);
};

