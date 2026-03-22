#include <LSM6DSOSensor.h>

static const uint8_t INT_PIN = 4;

static const int16_t GRANGE = 2000;
static const int16_t ARANGE = 16; 

static LSM6DSOSensor _lsm6dso(&Wire);

static bool bad(const LSM6DSOStatusTypeDef status)
{
    return status != LSM6DSO_OK;
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(0);

    Wire.begin();

    if (
            bad(_lsm6dso.begin()) ||
            bad(_lsm6dso.Enable_G())  ||
            bad(_lsm6dso.Enable_X()) ||
            bad(_lsm6dso.Set_X_FS(ARANGE)) ||
            bad(_lsm6dso.Set_G_FS(GRANGE))) {

        while (true) {
            printf("Initialization failed\n");
            delay(500);
        }
    }
}

void loop()
{
    uint8_t status = 0;

    _lsm6dso.Get_G_DRDY_Status(&status);

    if (status) {

        int16_t g[3] = {};
        _lsm6dso.Get_G_AxesRaw(g);

        int16_t a[3] = {};
        _lsm6dso.Get_X_AxesRaw(a);

        printf("gx=%d gy=%d gz=%d ax=%d ay=%d az=%d\n",
                g[0], g[1], g[2], a[0], a[1], a[2]);
    }
}
