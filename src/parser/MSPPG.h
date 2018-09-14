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


        static MSP_Message serialize_GET_RC_NORMAL(float c1, float c2, float c3, float c4, float c5, float c6);

        static MSP_Message serialize_GET_RC_NORMAL_Request();

        void set_GET_RC_NORMAL_Handler(class GET_RC_NORMAL_Handler * handler);

        static MSP_Message serialize_GET_ATTITUDE_RADIANS(float roll, float pitch, float yaw);

        static MSP_Message serialize_GET_ATTITUDE_RADIANS_Request();

        void set_GET_ATTITUDE_RADIANS_Handler(class GET_ATTITUDE_RADIANS_Handler * handler);

        static MSP_Message serialize_GET_ALTITUDE_METERS(float estalt, float vario);

        static MSP_Message serialize_GET_ALTITUDE_METERS_Request();

        void set_GET_ALTITUDE_METERS_Handler(class GET_ALTITUDE_METERS_Handler * handler);

        static MSP_Message serialize_GET_LOITER_RAW(byte agl, byte flowx, byte flowy);

        static MSP_Message serialize_GET_LOITER_RAW_Request();

        void set_GET_LOITER_RAW_Handler(class GET_LOITER_RAW_Handler * handler);

        static MSP_Message serialize_SET_MOTOR_NORMAL(float m1, float m2, float m3, float m4);

        static MSP_Message serialize_SET_ARMED(byte flag);

    private:

        class GET_RC_NORMAL_Handler * handlerForGET_RC_NORMAL;

        class GET_ATTITUDE_RADIANS_Handler * handlerForGET_ATTITUDE_RADIANS;

        class GET_ALTITUDE_METERS_Handler * handlerForGET_ALTITUDE_METERS;

        class GET_LOITER_RAW_Handler * handlerForGET_LOITER_RAW;

};


class GET_RC_NORMAL_Handler {

    public:

        GET_RC_NORMAL_Handler() {}

        virtual void handle_GET_RC_NORMAL(float c1, float c2, float c3, float c4, float c5, float c6)= 0;

};



class GET_ATTITUDE_RADIANS_Handler {

    public:

        GET_ATTITUDE_RADIANS_Handler() {}

        virtual void handle_GET_ATTITUDE_RADIANS(float roll, float pitch, float yaw)= 0;

};



class GET_ALTITUDE_METERS_Handler {

    public:

        GET_ALTITUDE_METERS_Handler() {}

        virtual void handle_GET_ALTITUDE_METERS(float estalt, float vario)= 0;

};



class GET_LOITER_RAW_Handler {

    public:

        GET_LOITER_RAW_Handler() {}

        virtual void handle_GET_LOITER_RAW(byte agl, byte flowx, byte flowy)= 0;

};

