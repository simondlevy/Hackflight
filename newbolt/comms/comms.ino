static HardwareSerial serial = HardwareSerial(PA3, PA2);

void setup() 
{
    serial.begin(115200);
}

void loop() 
{
    if (serial.available()) {

        static char msg[100];
        static uint8_t len;

        const char c = serial.read();

        if (c == '\n') {
            serial.printf("You said: %s\n", msg);
            memset(msg, 0, sizeof(msg));
            len = 0;
        }
        else {
            msg[len++] = c;
        }
    }
}
