void radioSetup()
{
    Serial1.begin(115000);
}

void serialEvent1(void)
{
    while (Serial1.available()) {
        DSM.handleSerialEvent(Serial1.read(), micros());
    }
}
