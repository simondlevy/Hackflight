extern "C" {

    class MSP {
        
        public:

            void com(
                    bool    armed,
                    int16_t angle[2],
                    int16_t heading,
                    int16_t motorsDisarmed[4],
                    int16_t rcData[RC_CHANS]);
    };
}

