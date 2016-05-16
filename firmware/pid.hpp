#define CONFIG_MAX_ANGLE_INCLINATION                500 /* 50 degrees */

extern "C" {

    // Mini-250 ==========================================

    static const int16_t CONFIG_ANGLE_TRIM[2] = {0, 0};

    class PID {

        private:

            static const uint8_t CONFIG_LEVEL_P          = 90;
            static const uint8_t CONFIG_LEVEL_I          = 10;

            static const uint8_t CONFIG_RATE_PITCHROLL_P = 40;
            static const uint8_t CONFIG_RATE_PITCHROLL_I = 30;
            static const uint8_t CONFIG_RATE_PITCHROLL_D = 23;

            static const uint8_t CONFIG_YAW_P            = 85;
            static const uint8_t CONFIG_YAW_I            = 45;

            static const int16_t  CONFIG_ROLL_TRIM       = 0;
            static const int16_t  CONFIG_PITCH_TRIM      = 0;

            uint8_t config_rate_p[3];
            uint8_t config_rate_i[3];
            uint8_t config_rate_d[3];

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
