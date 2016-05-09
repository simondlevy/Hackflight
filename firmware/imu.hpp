extern "C" {

    class IMU {

        public:

            void init();

            void getEstimatedAttitude(bool armed);
    };

}
