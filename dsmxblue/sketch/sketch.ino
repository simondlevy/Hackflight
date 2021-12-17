#include <BluetoothSerial.h>

static BluetoothSerial SerialBT;

void setup(void)
{
    Serial.begin(115200);
    SerialBT.begin("ESP32test3"); //Bluetooth device name
}

void loop(void)
{
    if (SerialBT.available()) {

        uint8_t b = SerialBT.read();

        if (b == '$') {
            printf("\n");
        }

        printf("x%02X\t", b);
    }
}
