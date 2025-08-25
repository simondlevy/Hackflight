static const uint8_t SPEED = 32;

static const uint8_t MAX = 128;

static const uint8_t MOTOR_1_PIN = PA1;
static const uint8_t MOTOR_2_PIN = PB11;
static const uint8_t MOTOR_3_PIN = PA15;
static const uint8_t MOTOR_4_PIN = PB9;

static volatile bool running;

static int8_t dir;

void serialEvent()
{
  while (Serial.available()) {
    Serial.read();
  }

  running = !running;
}

static uint8_t getSpeed()
{
    static uint8_t val;

    if (running) {

        val += dir;

        if (val >= MAX) {
            dir = -1;
        }

        if (val <= 1) {
            dir = +1;
        }
    }
    else {
        val = 0;
    }

    return val;
}

static void prompt()
{
    static uint32_t tprev;
    const uint32_t tcurr = millis();
    if (tcurr - tprev > 1000) {
      tprev = tcurr;
      Serial.print("Hit Enter to ");
      Serial.println(running ? "stop" : "start");
    }
}


void setup() 
{
    Serial.begin(115200);

    dir = +1;
}

void loop() 
{
    const uint8_t speed = getSpeed();

    analogWrite(MOTOR_1_PIN, speed);
    analogWrite(MOTOR_2_PIN, speed);
    analogWrite(MOTOR_3_PIN, speed);
    analogWrite(MOTOR_4_PIN, speed);

    prompt();

    delay(100);

}
