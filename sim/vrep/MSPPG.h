// AUTO-GENERATED CODE: DO NOT EDIT!!!\n\n'

static const int MAXBUF = 256;

typedef unsigned char byte;

class MSP_Message {

    friend class MSP_Parser;

    protected:

        MSP_Message() { }
        byte bytes[MAXBUF];
        int pos;
        int len;

    public:

        byte start();
        bool hasNext();
        byte getNext();

};

class MSP_Parser {

    private:

        int state;
        byte message_direction;
        byte message_id;
        byte message_length_expected;
        byte message_length_received;
        byte message_buffer[MAXBUF];
        byte message_checksum;

    public:

        MSP_Parser();

        void parse(byte b);


        static MSP_Message serialize_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8);

        static MSP_Message serialize_RC_Request();

        void set_RC_Handler(class RC_Handler * handler);

        static MSP_Message serialize_SET_HEAD(short head);

        static MSP_Message serialize_SONARS(short back, short front, short left, short right);

        static MSP_Message serialize_SONARS_Request();

        void set_SONARS_Handler(class SONARS_Handler * handler);

        static MSP_Message serialize_ATTITUDE(short roll, short pitch, short yaw);

        static MSP_Message serialize_ATTITUDE_Request();

        void set_ATTITUDE_Handler(class ATTITUDE_Handler * handler);

        static MSP_Message serialize_SET_RAW_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8);

        static MSP_Message serialize_SET_MOTOR(byte m1, byte m2, byte m3, byte m4);

        static MSP_Message serialize_ALTITUDE(int altitude, short vario);

        static MSP_Message serialize_ALTITUDE_Request();

        void set_ALTITUDE_Handler(class ALTITUDE_Handler * handler);

        static MSP_Message serialize_SONAR(short distance);

        static MSP_Message serialize_SONAR_Request();

        void set_SONAR_Handler(class SONAR_Handler * handler);

    private:

        class RC_Handler * handlerForRC;

        class SONARS_Handler * handlerForSONARS;

        class ATTITUDE_Handler * handlerForATTITUDE;

        class ALTITUDE_Handler * handlerForALTITUDE;

        class SONAR_Handler * handlerForSONAR;

};


class RC_Handler {

    public:

        RC_Handler() {}

        virtual void handle_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8){ }

};



class SONARS_Handler {

    public:

        SONARS_Handler() {}

        virtual void handle_SONARS(short back, short front, short left, short right){ }

};



class ATTITUDE_Handler {

    public:

        ATTITUDE_Handler() {}

        virtual void handle_ATTITUDE(short roll, short pitch, short yaw){ }

};



class ALTITUDE_Handler {

    public:

        ALTITUDE_Handler() {}

        virtual void handle_ALTITUDE(int altitude, short vario){ }

};



class SONAR_Handler {

    public:

        SONAR_Handler() {}

        virtual void handle_SONAR(short distance){ }

};

