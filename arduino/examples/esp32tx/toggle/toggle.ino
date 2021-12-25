static const uint8_t DIN_PIN = 4;

void setup(){
    pinMode(DIN_PIN, INPUT_PULLUP);
    Serial.begin(115200);
}

void loop(){
    int value;
    
    value = digitalRead(DIN_PIN);
    Serial.println(value);
    delay(1000);
}
