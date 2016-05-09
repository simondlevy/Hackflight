extern "C" {

    static const int INBUF_SIZE = 128;

    typedef enum serialState_t {
        IDLE,
        HEADER_START,
        HEADER_M,
        HEADER_ARROW,
        HEADER_SIZE,
        HEADER_CMD
    } serialState_t;

    typedef  struct mspPortState_t {
        uint8_t checksum;
        uint8_t indRX;
        uint8_t inBuf[INBUF_SIZE];
        uint8_t cmdMSP;
        uint8_t offset;
        uint8_t dataSize;
        serialState_t c_state;
    } mspPortState_t;

    class MSP {

        private:

            mspPortState_t portState;

            void serialize8(uint8_t a);
            void serialize16(int16_t a);
            uint8_t read8(void);
            uint16_t read16(void);
            uint32_t read32(void);
            void serialize32(uint32_t a);
            void headSerialResponse(uint8_t err, uint8_t s);
            void headSerialReply(uint8_t s);
            void headSerialError(uint8_t s);
            void tailSerialReply(void);
            void init(void);

        public:

            void com(
                    bool    armed,
                    int16_t angle[2],
                    int16_t heading,
                    int16_t motorsDisarmed[4],
                    int16_t rcData[RC_CHANS]);

    }; // class MSP

} // extern "C"
