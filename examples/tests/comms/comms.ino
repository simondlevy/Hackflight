void setup() 
{
    Serial5.begin(115200);
}

void loop() 
{
    if (Serial5.available()) {

        static char msg[100];
        static uint8_t len;

        const char c = Serial5.read();

        if (c == '\n') {
            Serial5.printf("You said: %s\n", msg);
            memset(msg, 0, sizeof(msg));
            len = 0;
        }
        else {
            msg[len++] = c;
        }
    }
}
