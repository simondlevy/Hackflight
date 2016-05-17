#define CONFIG_MINCOMMAND     1000
#define CONFIG_YAW_DIRECTION     1

extern "C" {

    class Mixer {

        private:

            Board * _board;
            RC * _rc;
            PID * _pid;
        
        public:

            int16_t  motorsDisarmed[4];

            void init(Board * board, RC * rc, PID * pid);

            void update(bool armed);
    };

}
