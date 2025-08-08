void setup() 
{

    Serial.begin(0);

    Serial5.begin(115200);
}

void loop() 
{

    Serial5.printf("Hello!");
}
