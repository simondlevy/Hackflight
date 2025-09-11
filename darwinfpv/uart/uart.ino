static const uint8_t LED_PIN = PC13; // XXX

void setup() 
{
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(115200);
}

void loop() 
{
    digitalWrite(LED_PIN, HIGH);  
    delay(1000);                     
    digitalWrite(LED_PIN, LOW); 
    delay(1000);
}
