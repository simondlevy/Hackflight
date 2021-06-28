static uint8_t b;

void setup() 
{
    Serial2.begin(115200);

    b = 0;
}

void loop() 
{
    char c = ('A' + b);

    Serial2.write(c);

    b = (b + 1) % 26;

    delay(100);
}
