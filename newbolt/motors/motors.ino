#include <dshot_stm32f4.h>

#include <vector>

static const uint8_t PIN = PB11;

static std::vector<uint8_t> pins = {PIN};

static const uint32_t FREQUENCY = 8000;

static Stm32F4Dshot dshot;


extern "C" void DMA2_Stream1_IRQHandler(void) 
{
    dshot.handleDmaIrqStream1();
}

extern "C" void DMA2_Stream2_IRQHandler(void) 
{
    dshot.handleDmaIrqStream2();
}

static float motorval;

void serialEvent(void)
{
    if (Serial.available()) {
        Serial.read();
        motorval = motorval == 0 ? 0.1 : 0;
    }
}

static void run(const uint32_t usec)
{
    static uint32_t prev;

    if (usec-prev > 1000000/FREQUENCY) {
        prev = usec;
        dshot.write(&motorval);
    }
}

static void prompt(const uint32_t usec)
{
    static uint32_t prev;

    if (usec-prev > 1000000) {
        prev = usec;
        Serial.println(
                motorval == 0 ?
                "After removing propellers, hit Enter to start motor" :
                "Hit Enter to stop motor"
                );
    }
}

void setup(void)
{
    Serial.begin(115200);

    dshot.begin(pins);
}

void loop(void)
{
    const auto usec = micros();

    prompt(usec);

    run(usec);
}
