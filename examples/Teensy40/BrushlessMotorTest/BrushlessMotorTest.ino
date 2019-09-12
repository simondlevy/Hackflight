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
    static float val;
    static int dir = +1;

    int intval = (int)(MINVAL + val * (MAXVAL-MINVAL));

    motor1.set(intval);
    motor2.set(intval);
    motor3.set(intval);
    motor4.set(intval);

    Serial.printf("%f %d\n", val, intval);

    if (val <= 0) dir = +1;
    if (val >= 1) dir = -1;
    val += dir * .01;

    delay(100);
}
