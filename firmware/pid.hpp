#define CONFIG_MAX_ANGLE_INCLINATION                500 /* 50 degrees */

extern "C" {

    // Mini-250 ==========================================

    static const uint8_t CONFIG_LEVEL_P  = 90;
    static const uint8_t CONFIG_LEVEL_I  = 10;
    static const uint8_t CONFIG_LEVEL_D  = 100;

    // roll, pitch, yaw
    static const uint8_t CONFIG_AXIS_P[3] = {40, 40, 85};
    static const uint8_t CONFIG_AXIS_I[3] = {30, 30, 45};
    static const uint8_t CONFIG_AXIS_D[3] = {23, 23, 0};

    static const int16_t CONFIG_ANGLE_TRIM[2] = {0, 0};

    // Micro ==============================================

    /*
    static const uint8_t CONFIG_LEVEL_P  = 20;
    static const uint8_t CONFIG_LEVEL_I  = 0;
    static const uint8_t CONFIG_LEVEL_D  = 0;

    // roll, pitch, yaw
    static const uint8_t CONFIG_AXIS_P[3] = {20,20,40};
    static const uint8_t CONFIG_AXIS_I[3] = {0,0,0};
    static const uint8_t CONFIG_AXIS_D[3] = {0,0,0};

    static const int16_t CONFIG_ANGLE_TRIM[2] = {-100, 0};
    */

    // General =============================================

    static const bool CONFIG_HORIZON_MODE = true;

    class PID {

        private:

            int16_t lastGyro[3];
            int32_t delta1[3]; 
            int32_t delta2[3];
            int32_t errorGyroI[3];
            int32_t errorAngleI[2];

        public:

            int16_t axisPID[3];

            void init(void);

            void update(RC * rc, IMU * imu);

            void resetIntegral(void);
    }; 

} // extern "C"
