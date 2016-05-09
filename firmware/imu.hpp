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
