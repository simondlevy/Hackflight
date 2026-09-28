void setup()
{
    Serial3.begin(115200);
}

void loop()
{
    static uint8_t k_;

    const uint8_t c = 'A' + k_;

    Serial3.write(c);

    k_ = (k_ + 1) % 26;

    delay(2);
}
