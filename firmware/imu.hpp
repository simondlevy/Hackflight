extern "C" {

    class IMU {
        
        private:

            uint16_t acc1G;
            float    gyroScale;

        public:

            void init();

            void getEstimatedAttitude( bool armed, 
                    float anglerad[3], 
                    int16_t gyroADC[3],
                    uint16_t & calibratingA,
                    uint16_t & calibratingG);
    };

}
