#include <Arduino.h>
#include <Servo.h>

static const int MINVAL = 50;
static const int MAXVAL = 150;

class Motor {

    private:

        Servo _servo;

    public:

        Motor(uint8_t pin) 
        {
            _servo.attach(pin);
        }

        void begin(void)
        {
            _servo.write(MAXVAL);
            delay(250);
            _servo.write(MINVAL);
        }

        void set(int val)
        {
            _servo.write(val);
        }
};

static Motor motor1(14);
static Motor motor2(15);
static Motor motor3(9);
static Motor motor4(2);

void setup(void)
{
    motor1.begin();
    motor2.begin();
    motor3.begin();
    motor4.begin();

    Serial.begin(115200);

    delay(1000);
}

void loop(void)
{
    static int val = MINVAL;
    static int dir = +1;

    static float newval;
    static int newdir = +1;

    motor1.set(val);
    motor2.set(val);
    motor3.set(val);
    motor4.set(val);

    int newint = (int)(MINVAL + newval * (MAXVAL-MINVAL));

    Serial.printf("%d %f %d\n", val, newval, newint);

    val += dir;
    if (val == MINVAL) dir = +1;
    if (val == MAXVAL) dir = -1;

    if (newval <= 0) newdir = +1;
    if (newval >= 1) newdir = -1;
    newval += newdir * .01;


    delay(100);
}
