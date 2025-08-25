#include <Wire.h>   

//static TwoWire wire(PB9, PB8);

void setup()
{
    Serial.begin(115200);

    Wire.begin();
    delay(100);

}

static void scan(TwoWire & wire, const char * name)
{

    Serial.print("Scanning ");
    Serial.print(name);
    Serial.println (" ...");

    int nDevices = 0;

    for(byte address = 1; address < 127; address++ ) 
    {

        wire.beginTransmission(address);

        byte error = wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address<16) 
                Serial.print("0");
            Serial.println(address,HEX);

            nDevices++;
        }
        else if (error==4) 
        {
            Serial.print("Unknown error at address 0x");
            if (address<16) 
                Serial.print("0");
            Serial.println(address,HEX);
        }    
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n"); 

    delay(1000);
}

void loop()
{  
    scan(Wire, "wire");
}
