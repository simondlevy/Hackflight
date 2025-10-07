static const uint8_t TXD1 = 14;
static const uint8_t RXD1 = 4;

HardwareSerial mySerial(1);

int counter = 0;

void setup() 
{
    mySerial.begin(115200, SERIAL_8N1, RXD1, TXD1);
}

void loop() 
{
    static uint8_t value;

    mySerial.write(value);

    value = (value + 1) % 256;
}
