#define CONFIG_ACC_LPF_FACTOR     4
#define CONFIG_ACCZ_DEADBAND      40
#define CONFIG_ACCXY_DEADBAND     40
#define CONFIG_ACCZ_LPF_CUTOFF    5.0F
#define CONFIG_GYRO_CMPF_FACTOR   600    
#define CONFIG_GYRO_CMPFM_FACTOR  250  
#define CONFIG_MORON_THRESHOLD     32

extern "C" {

    class IMU {
        
        private:

            uint16_t acc1G;
            float    fcAcc;
            float    gyroScale;

        public:

            int16_t gyroADC[3];

            void init(void);

            void getEstimatedAttitude(bool armed, float anglerad[3], uint16_t & calibratingA, uint16_t & calibratingG);
    };

}
