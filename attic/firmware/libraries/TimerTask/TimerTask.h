#include <Arduino.h>

class TimerTask {

    private:

        void (*func)();
        int limit;
        unsigned long tick;

    public:

        TimerTask(void (*func)(), int freq) {

            this->func = func;
            this->limit = 1000000L / freq;

            this->tick = micros();
        }

        void update() {

            if ((micros() - this->tick) > this->limit) {

                this->func();
                this->tick = micros();
            }
        }

};



