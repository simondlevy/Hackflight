#include <BluetoothSerial.h>
#include <TinyPICO.h>

static TinyPICO tp = TinyPICO(); 

static BluetoothSerial serialBt;

void setup() 
{
  serialBt.begin("Esp32-BT");
}

void loop() 
{
  if (serialBt.available()) {

    const uint8_t cmd = serialBt.read();

    uint8_t r = 0;
    uint8_t g = 0;

    if (cmd == '1') {
        r = 255;
    }

    else if (cmd == '2') {
        g = 255;
    }

    tp.DotStar_SetPixelColor(r, g, 0); 
    
  }

  delay(20);
}
