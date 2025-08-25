static const uint8_t SPEED = 32;

static const uint8_t PINS[4] = {PA1, PB11, PA15, PB9};

void setup() 
{
}

void loop() 
{
    analogWrite(PINS[3], SPEED);
}
