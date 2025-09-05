#include <BluetoothSerial.h>
#include <TinyPICO.h>

static const uint8_t LED_SIGNAL_PIN = 22;

static TinyPICO tp = TinyPICO(); 

static BluetoothSerial serialBt;

static void setLed(const bool on)
{
    tp.DotStar_SetPixelColor(on ? 255 : 0, 0, 0); 
}

void setup() 
{
    serialBt.begin("Esp32-BT");

    Serial.begin(115200);

    setLed(false);

    pinMode(LED_SIGNAL_PIN, INPUT);
}

void loop() 
{
    /*
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
    }*/

    setLed(digitalRead(LED_SIGNAL_PIN));
}
