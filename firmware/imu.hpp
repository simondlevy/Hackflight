extern "C" {

    class IMU {

        public:

            void init();

            void getEstimatedAttitude(bool armed, float anglerad[3], int16_t gyroADC[3]);
    };

}
