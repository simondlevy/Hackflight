#include <BluetoothSerial.h>
//#include <TinyPICO.h>

//static TinyPICO tp = TinyPICO(); 

static const uint8_t RX1_PIN = 32;
static const uint8_t TX1_PIN = 33;

static BluetoothSerial serialBt;

HardwareSerial uart(1);

/*
static void setLed(const bool on)
{
    tp.DotStar_SetPixelColor(on ? 255 : 0, 0, 0); 
}*/

void setup() 
{
    Serial.begin(115200);

    //serialBt.begin("Esp32-Bazinga");

    uart.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);

    //setLed(false);
}

void loop() 
{
    if (uart.available()) {
        Serial.println((char)uart.read());
    }

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
}
