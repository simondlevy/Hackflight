static const uint8_t RX1_PIN = 2; // unused;
static const uint8_t TX1_PIN = 4; 

static void error(const char * message)
{
    while (true) {
        Serial.println(message);
        delay(500);
    }
}

void setup() 
{
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Begin serial connection to flight controller
    Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
}

void loop() 
{
    static uint8_t k;

    const char c =  'A' + k;

    Serial1.write(c);

    k = (k + 1) % 26;
}
