static int count_;

void serialEvent3()
{
    count_++;

    while (Serial3.available()) {
        Serial3.read();
    }
}

void setup()
{
    Serial3.begin(115200);
}

void loop()
{
    printf("%d\n", count_);
}
