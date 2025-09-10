#include <BluetoothSerial.h>
#include <TinyPICO.h>

static TinyPICO tp = TinyPICO(); 

static BluetoothSerial serialBt;

static void setLed(const bool on)
{
    tp.DotStar_SetPixelColor(on ? 255 : 0, 0, 0); 
}

void setup() 
{
    Serial.begin(115200);

    serialBt.begin("Esp32-Bazinga");

    setLed(false);
}

void loop() 
{
    if (serialBt.available()) {

        const uint8_t cmd = serialBt.read();

        if (cmd == '0') {
            Serial.println("off");
            setLed(false);
        }

        else if (cmd == '1') {
            Serial.println("on");
            setLed(true);
        }
    }
}
