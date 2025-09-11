//static const uint8_t LED_PIN = PC8; // XXX

void setup() 
{
    //pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
}

void loop() 
{
    /*
       digitalWrite(LED_BUILTIN, HIGH);  
       delay(1000);                     
       digitalWrite(LED_BUILTIN, LOW); 
       delay(1000);*/

    Serial.println(micros());
}
