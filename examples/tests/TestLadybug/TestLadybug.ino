static uint32_t avail;
static uint8_t buf[256];

void serialEvent2(void)
{
    avail = Serial2.available();

    Serial2.readBytes(buf, avail);
}

void setup(void)
{
    Serial.begin(115200);

    Serial2.begin(115200);

}

void loop(void)
{
    for (uint32_t k=0; k<avail; ++k) {
        Serial.println((char)buf[k]);
    }

    delay(100);
}

namespace std {
    void __throw_bad_alloc() {
        while (true) {
            Serial.println("Unable to allocate memory");
            delay(500);
        }
    }
}
