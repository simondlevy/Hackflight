static const int DELAY = 500;
static const uint8_t PIN = 15;

void setup() 
{
    pinMode(PIN, OUTPUT);
}

void loop() 
{
    digitalWrite(PIN, HIGH);  
    delay(DELAY);             
    digitalWrite(PIN, LOW); 
    delay(DELAY);           
}
