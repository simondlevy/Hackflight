void setup(void)
{
    Serial.begin(115200);
}

void loop(void)
{
    Serial.println(micros());
}
