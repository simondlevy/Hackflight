static const uint8_t _3V3_INPUT_PIN = A0;

void setup() 
{
}

void loop() 
{
    printf("3.3V=%d\n", analogRead(_3V3_INPUT_PIN));
}
