static const uint8_t VOLTAGE_INPUT_PIN = A9;

void setup() 
{
}

void loop() 
{
    printf("%d\n", analogRead(VOLTAGE_INPUT_PIN));
}
