#include <LSM6DSOSensor.h>

static const uint8_t INT_1_PIN = 4;

LSM6DSOSensor accGyr(&Wire);

//Interrupts.
volatile int mems_event = 0;

char report[256];

void INT1Event_cb();
void sendOrientation();

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(0);

    Wire.begin();

    attachInterrupt(INT_1_PIN, INT1Event_cb, RISING);

    accGyr.begin();
    accGyr.Enable_X();
    accGyr.Enable_6D_Orientation(LSM6DSO_INT1_PIN);
}

void loop()
{

    if (mems_event) {

        mems_event=0;
        LSM6DSO_Event_Status_t status;
        accGyr.Get_X_Event_Status(&status);
        if (status.D6DOrientationStatus)
        {
            sendOrientation();
            
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);  
        }
    }
}

void INT1Event_cb()
{
    mems_event = 1;
}

void sendOrientation()
{
    uint8_t xl = 0;
    uint8_t xh = 0;
    uint8_t yl = 0;
    uint8_t yh = 0;
    uint8_t zl = 0;
    uint8_t zh = 0;

    accGyr.Get_6D_Orientation_XL(&xl);
    accGyr.Get_6D_Orientation_XH(&xh);
    accGyr.Get_6D_Orientation_YL(&yl);
    accGyr.Get_6D_Orientation_YH(&yh);
    accGyr.Get_6D_Orientation_ZL(&zl);
    accGyr.Get_6D_Orientation_ZH(&zh);

    if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 )
    {
        sprintf( report, "\r\n  ________________  " \
                "\r\n |                | " \
                "\r\n |  *             | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |________________| \r\n" );
    }

    else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
    {
        sprintf( report, "\r\n  ________________  " \
                "\r\n |                | " \
                "\r\n |             *  | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |________________| \r\n" );
    }

    else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 )
    {
        sprintf( report, "\r\n  ________________  " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |  *             | " \
                "\r\n |________________| \r\n" );
    }

    else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
    {
        sprintf( report, "\r\n  ________________  " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |                | " \
                "\r\n |             *  | " \
                "\r\n |________________| \r\n" );
    }

    else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 )
    {
        sprintf( report, "\r\n  __*_____________  " \
                "\r\n |________________| \r\n" );
    }

    else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 )
    {
        sprintf( report, "\r\n  ________________  " \
                "\r\n |________________| " \
                "\r\n    *               \r\n" );
    }

    else
    {
        sprintf( report, "None of the 6D orientation axes is set in LSM6DSO - accelerometer.\r\n" );
    }

    Serial.print(report);
}
