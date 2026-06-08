#include <dshot-teensy4.hpp>  

static constexpr uint8_t VOLTAGE_INPUT_PIN = A9;
static constexpr float VOLTAGE_SCALEUP = 4.29;

static DshotTeensy4 _motors = DshotTeensy4({2});

static float MAX = 0.5;
static float INC = 1e-5;

static bool running;

static float val;
static float dir;

void serialEvent()
{
  while (Serial.available()) {
    Serial.read();
  }

  running = !running;
}

static void prompt()
{
    static uint32_t tprev;
    const uint32_t tcurr = millis();
    if (tcurr - tprev > 1000) {
      tprev = tcurr;
    }
}

static float inputGet()
{
    prompt();

    if (running) {

        val += INC * dir;

        if (val >= MAX) {
            dir = -1;
        }

        if (val <= 0) {
            dir = +1;
        }
    }
    else {
        val = 0;
    }

    return val;
}

void setup()
{
    delay(500);

    dir = +1;

    _motors.begin(); 
}

void loop()
{
    const float inp = inputGet();

    float motorvals[4] = {inp, inp, inp, inp};

    _motors.run(inp > 0, motorvals);

    printf("%3.3f\n",analogRead(VOLTAGE_INPUT_PIN) * 3.3 * VOLTAGE_SCALEUP / 1023);

    delay(1);
}

