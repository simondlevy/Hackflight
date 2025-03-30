static const uint8_t GREEN_LED = PA13;
static const uint8_t BLUE_LED  = PA14;

static const uint8_t pins[2] = {BLUE_LED, GREEN_LED};
static const uint8_t polarities[2] = {0, 0};

static uint8_t active;

void setup() 
{
    for (uint8_t k=0; k<2; ++k) {
        pinMode(pins[k], OUTPUT);
        digitalWrite(pins[k], 1-polarities[k]);
    }

    active = 0;
}

void loop() 
{

    digitalWrite(pins[active], polarities[active]);
    delay(200);
    digitalWrite(pins[active], 1-polarities[active]);
    delay(200);

    active = (active + 1) % 2;
}
