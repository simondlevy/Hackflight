#include <LSM6DSOSensor.h>

static const uint8_t INT_1_PIN = 4;

static LSM6DSOSensor accGyr(&Wire);

static volatile bool got_interrupt;

static void INT1Event_cb()
{
    got_interrupt = true;
}

static void sendOrientation()
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

    printf("xl=%d xh=%d yl=%d yh=%d zl=%d zh=%d\n", xl, xh, yl, yh, zl, zh);
}

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

    if (got_interrupt) {

        got_interrupt = false;

        LSM6DSO_Event_Status_t status;

        accGyr.Get_X_Event_Status(&status);

        if (status.D6DOrientationStatus) {

            sendOrientation();

            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);  
        }
    }
}

