static const uint8_t BLUE_L  = PD2;
static const uint8_t GREEN_L = PC1;
static const uint8_t RED_L   = PC0;
static const uint8_t GREEN_R = PC2;
static const uint8_t RED_R   = PC3;

static const uint8_t pins[5] = {BLUE_L, GREEN_L, RED_L, GREEN_R, RED_R};
static const uint8_t polarities[5] = {1, 0, 0, 0, 0};

static uint8_t active;

void setup() 
{
    for (uint8_t k=0; k<5; ++k) {
        pinMode(pins[k], OUTPUT);
    }

    active = 0;
}

void loop() 
{
    digitalWrite(pins[active], polarities[active]);
    delay(500);
    digitalWrite(pins[active], 1-polarities[active]);
    delay(500);

    active = (active + 1) % 5;
}
