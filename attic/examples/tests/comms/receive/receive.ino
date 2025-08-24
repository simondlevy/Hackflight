void setup() 
{
    Serial5.begin(115200);
}

void loop() 
{
    static uint8_t k;

    Serial5.write(65 + k);

    k = (k + 1) % 26;

    delay(250);
}
