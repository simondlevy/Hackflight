static const uint8_t NLEDS = 5;

static const uint8_t BLUE_L  = PD2;
static const uint8_t GREEN_L = PC1;
static const uint8_t RED_L   = PC0;
static const uint8_t GREEN_R = PC2;
static const uint8_t RED_R   = PC3;

static const uint8_t pins[NLEDS] = {BLUE_L, GREEN_L, RED_L, GREEN_R, RED_R};
static const uint8_t polarities[NLEDS] = {1, 0, 0, 0, 0};
static const char * names[NLEDS] = {"BLUE_L", "GREEN_L", "RED_L", "GREEN_R", "RED_R"};

static uint8_t active;

void setup() 
{
    Serial.begin(115200);

    for (uint8_t k=0; k<NLEDS; ++k) {
        pinMode(pins[k], OUTPUT);
        digitalWrite(pins[k], 1-polarities[k]);
    }

    active = 0;
}

void loop() 
{
    digitalWrite(pins[active], polarities[active]);
    delay(500);
    digitalWrite(pins[active], 1-polarities[active]);
    delay(500);

    Serial.println(names[active]);

    active = (active + 1) % 5;
}
