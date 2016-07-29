class SerialConnection {

    public:

        SerialConnection(const char * portname, int baudrate=9600, bool blocking=true, int parity=0);

        int readBytes(char * buf, int size);

        int writeBytes(char * buf, int size);

        void closeConnection(void);

    private:

        int fd;
};
