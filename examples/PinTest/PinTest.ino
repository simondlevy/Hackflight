void setup(void)
{
    Serial1.begin(115200, SERIAL_8N1, 15, 27);
}

void loop(void)
{
    static uint8_t count;

    Serial1.write(count);

    count = (count + 1) % 256;

    delay(5);
}


