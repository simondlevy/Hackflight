#include <BluetoothSerial.h>
#include <TinyPICO.h>

static const uint8_t LED_SIGNAL_PIN = 22;

static const uint8_t UART_RX_PIN = 21;
static const uint8_t UART_TX_PIN = 32;

static TinyPICO tp = TinyPICO(); 

static BluetoothSerial serialBt;

static HardwareSerial serialUart(2);

static void setLed(const bool on)
{
    tp.DotStar_SetPixelColor(on ? 255 : 0, 0, 0); 
}

HardwareSerial gpsSerial(2);

void setup() 
{
    Serial.begin(115200);

    serialBt.begin("Esp32-BT");

    serialUart.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

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

    while (serialUart.available()) {
        Serial.print((char)serialUart.read());
    }

    setLed(digitalRead(LED_SIGNAL_PIN));
}
