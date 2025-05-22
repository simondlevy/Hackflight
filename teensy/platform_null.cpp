#include <tasks/imu.hpp>

// General -------------------------------------------------------------------

void debug(const char * msg)
{
}

void error(const char * msg)
{
}

void assertFail(char *exp, char *file, int line)
{
}

void init_platform_specific()
{
    usecTimerInit();
}

// VL53L1 --------------------------------------------------------------------

void vl53l1_init()
{
}

float vl53l1_read()
{
    return 0;
}


// Flowdeck ------------------------------------------------------------------

const uint8_t get_flowdeck_cs_pin()
{
    return 11;
}

// IMU -----------------------------------------------------------------------

void ImuTask::readGyroRaw(Axis3i16* dataOut)
{
}

void ImuTask::readAccelRaw(Axis3i16 * dataOut)
{
}

void ImuTask::deviceInit(void)
{
}

// UART -------------------------------------------------------------------

bool uartReadByte(uint8_t * byte)
{
    vTaskDelay(1);

    return false;
}

void uartWriteByte(const uint8_t byte)
{
}


// Motors --------------------------------------------------------------------

void motorsInit()
{
}

int motorsGetRatio(uint32_t id)
{
    return 0;
}

bool motorsTest(void)
{
    return true;
}

void motorsSetRatios(const uint16_t ratios[])
{

}

extern "C" {

    void motorsStop()
    {
    }
}


// Main --------------------------------------------------------------------

int main() 
{
    void systemInit();
    systemInit();

    while(true) {
    }

    return 0;
}
