#define CONFIG_MINCOMMAND     1000
#define CONFIG_YAW_DIRECTION     1

extern "C" {

    class Mixer {
        
        public:

            int16_t  motorsDisarmed[4];

            void init(void);

            void writeMotors(bool armed, 
                    int16_t  axisPID[3], 
                    int16_t  rcCommand[4], 
                    bool throttleIsDown);
    };

}
