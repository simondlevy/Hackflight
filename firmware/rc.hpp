#define CONFIG_RC_EXPO_8                            65
#define CONFIG_RC_RATE_8                            90
#define CONFIG_MIDRC                                1490
#define CONFIG_THR_MID_8                            50
#define CONFIG_THR_EXPO_8                           0

#define PITCH_LOOKUP_LENGTH    7
#define THROTTLE_LOOKUP_LENGTH 12

extern "C" {

    class RC {

        private:

            int16_t dataAverage[8][4];
            int32_t averageIndex;
            int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];     // lookup table for expo & RC rate PITCH+ROLL
            int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE

        public:

            int16_t  command[4];

            int16_t  data[RC_CHANS];

            void init(void);

            void compute(void);

            void computeExpo(void);
    };
}
