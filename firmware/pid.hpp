#define CONFIG_MAX_ANGLE_INCLINATION                500 /* 50 degrees */

extern "C" {

    static const bool CONFIG_HORIZON_MODE = true;

    static const uint8_t CONFIG_LEVEL_P  = 90;
    static const uint8_t CONFIG_LEVEL_I  = 10;
    static const uint8_t CONFIG_LEVEL_D  = 100;

    // roll, pitch, yaw
    static const uint8_t CONFIG_AXIS_P[3] = {40, 40, 85};
    static const uint8_t CONFIG_AXIS_I[3] = {30, 30, 45};
    static const uint8_t CONFIG_AXIS_D[3] = {23, 23, 0};

    static const int16_t CONFIG_ANGLE_TRIM[2] = {-50, -25};

    class PID {

        private:

            int16_t lastGyro[3];
            int32_t delta1[3]; 
            int32_t delta2[3];
            int32_t  errorGyroI[3];
            int32_t  errorAngleI[2];

        public:

            void init(void);

            void compute(int16_t rcCommand[4], int16_t angle[3], int16_t gyroADC[3], int16_t axisPID[3]); 

            void resetIntegral(void);
    }; 

} // extern "C"
