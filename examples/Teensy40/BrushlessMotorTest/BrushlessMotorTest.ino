#include <Arduino.h>
#include <Servo.h>

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
            _servo.write(140);
            delay(250);
            _servo.write(50);
        }

        void set(uint8_t val)
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
    static int val = 50;
    static int dir = +1;

    motor1.set(val);
    motor2.set(val);
    motor3.set(val);
    motor4.set(val);

    Serial.println(val);

    val += dir;

    if (val == 50) dir = +1;
    if (val == 140) dir = -1;

    delay(100);
}
