static HardwareSerial uart = HardwareSerial(PA3, PA2);

void setup() 
{
    Serial.begin(115200);

    uart.begin(115200);
}

void loop() 
{
    static uint8_t c;

    uart.write('A' + c);

    c = (c + 1) % 26;

    delay(50);
}
