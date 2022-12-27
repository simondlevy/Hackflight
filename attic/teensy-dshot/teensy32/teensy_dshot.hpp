// Adapted from https://forum.pjrc.com/archive/index.php/t-45083.html

#include <DMAChannel.h>
#include <array>

#if defined(__IMXRT1062__) // teensy 4.0
  #define F_TMR F_BUS_ACTUAL
#else // teensy 3.X
  #define F_TMR F_BUS
#endif

static const uint16_t short_pulse = uint64_t(F_TMR) * 625 / 1000000000;
static const uint16_t long_pulse  = uint64_t(F_TMR) * 1250 / 1000000000;
static const uint16_t bit_length  = uint64_t(F_TMR) * 1666 / 1000000000; // 1670

class DshotMotor {

    private:

        DMAChannel dma;
        std::array<volatile uint16_t, 18> dma_source;
        uint8_t pin[8]; // Up to 8 motor pins;
        uint8_t cnt = 0;
        volatile uint32_t dmaDoneAt, xferTimeMicros;

        void dmaSetup() 
        {
            dma.sourceBuffer(dma_source.data(), sizeof(dma_source));
#if defined(__IMXRT1062__) 
            dma.triggerAtHardwareEvent(DMAMUX_SOURCE_FLEXPWM4_WRITE0); // XXX FLEXPWMX_WRITE_Y
#else
            dma.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM0_CH7) ;
#endif
            dma.disableOnCompletion();
        }

        void dshotSend(uint8_t pin) 
        {
            switch (pin) {
                case 22: 
#if defined(__IMXRT1062__)
#else
                    dma.destination((uint16_t&) FTM0_C0V); 
                    CORE_PIN22_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; 
#endif
                    break;
                default: 
                    return;
            };

#if defined(__IMXRT1062__) 
#else
            FTM0_SC = 0;  //disable FTM0
            FTM0_CNT = 0;
            FTM0_CNTIN = 0;
            FTM0_MOD = bit_length; //edge aligned PWM output for channel 0
            FTM0_C0SC = FTM_CSC_MSB | FTM_CSC_ELSB;
            FTM0_C0V = 0; //channel 7 triggers DMA
            FTM0_C7SC = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA;  //output compare, enable DMA trigger
            FTM0_C7V = 0;
            FTM0_SC = FTM_SC_CLKS(1);  //enable timer with busclock
#endif
            dma.enable();
            dmaDoneAt = micros() + xferTimeMicros;
        }

        uint16_t prepareDshotPacket(const uint16_t value, uint16_t telemBit = 0)
        {
            uint16_t packet = (value << 1) | telemBit;
            int csum = 0, csum_data = packet;
            for (int i = 0; i < 3; i++) {
                csum ^= csum_data; // xor data by nibbles
                csum_data >>= 4;
            }
            return (packet << 4) | (csum & 0xf); // append checksum
        }

        void dshotSend(uint8_t pin, uint16_t dsWord) 
        {
            volatile int32_t toWait = dmaDoneAt - micros();
            if (toWait > 0) {
                // If clock rolls over we can get a weird number.
                delayMicroseconds((uint32_t)toWait < xferTimeMicros ? toWait : xferTimeMicros); 
            }
            for (int i=0; i<16; i++) {
                dma_source[i] = (dsWord<<i & 0x8000) ? long_pulse : short_pulse; // MSB first
            }
            dshotSend(pin);
        }

        uint16_t sendDshotCmd(uint8_t pin, int cmd) 
        {
            uint16_t dsWord = prepareDshotPacket(cmd);
            dshotSend(pin, dsWord);
            return dsWord;
        }

    public:

        bool armed = false;

        DshotMotor(int motorCount, uint8_t M[]) 
        {
            for(int i=0; i<motorCount; i++) {
                switch(M[i]) {
                    case 22: 
                        pin[i] = M[i]; 
                        break; 
                    default: 
                        Serial.print("Invalid pin for DShot: "); 
                        Serial.println(M[i]); 
                        return; // Throw something
                }
            }
            cnt = motorCount;
            int bit_period = bit_length * uint64_t(1000000000) / uint64_t(F_TMR);
            xferTimeMicros = (16 + 2) * bit_period / 1000;
            dmaSetup();
        }

        void armMotorESC() 
        {
            for (int i=0; i<=2000; i+=5) { 
                setRunSpeed(i); 
                delay(2); 
            } // Throttle up to max
            for (int i=2000; i>=0; i-=5) { 
                setRunSpeed(i); 
                delay(2); 
            } // Throttle zown to zero
            armed = true;
        }

        void setRunSpeed(int n, int escSpeed) 
        { 
            sendDshotCmd(pin[n], escSpeed + 48); 
        }

        void setRunSpeed(int escSpeed) 
        { 
            for(int i=0; i<4; i++) {
                setRunSpeed(i, escSpeed); 
            }
        }
};
