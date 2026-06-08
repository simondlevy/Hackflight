static constexpr uint8_t VOLTAGE_INPUT_PIN = A9;
static constexpr float VOLTAGE_SCALEUP = 4.29;

void setup() 
{
}

void loop() 
{
    printf("%3.3f\n", analogRead(VOLTAGE_INPUT_PIN) * 3.3 * VOLTAGE_SCALEUP / 1023);
}
