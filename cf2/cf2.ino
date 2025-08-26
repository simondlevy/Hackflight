static const uint8_t LED_PIN = PC0;

void setup() 
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
}

void loop() 
{
}
