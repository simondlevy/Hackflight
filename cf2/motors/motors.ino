static const uint8_t MOTOR_1 = PA1;
static const uint8_t MOTOR_2 = PB11;
static const uint8_t MOTOR_3 = PA15;
static const uint8_t MOTOR_4 = PB9;

void setup() 
{

}

void loop() 
{
    analogWrite(MOTOR_4, 200);
}
