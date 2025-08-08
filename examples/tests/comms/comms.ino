void serialEvent5()
{
    digitalWrite(LED_BUILTIN, Serial5.read() == '+');
}

void setup() 
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial5.begin(115200);
}

void loop() 
{
}
