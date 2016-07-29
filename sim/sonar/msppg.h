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


        static MSP_Message serialize_SET_HEAD(short head);

        static MSP_Message serialize_ATTITUDE(short angx, short angy, short heading);

        static MSP_Message serialize_ATTITUDE_Request();

        void set_ATTITUDE_Handler(class ATTITUDE_Handler * handler);

        static MSP_Message serialize_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8);

        static MSP_Message serialize_RC_Request();

        void set_RC_Handler(class RC_Handler * handler);

        static MSP_Message serialize_SET_RAW_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8);

        static MSP_Message serialize_ALTITUDE(int altitude, short vario);

        static MSP_Message serialize_ALTITUDE_Request();

        void set_ALTITUDE_Handler(class ALTITUDE_Handler * handler);

        static MSP_Message serialize_SLAM_POSE(short x, short y, short z, short theta);

        static MSP_Message serialize_SLAM_POSE_Request();

        void set_SLAM_POSE_Handler(class SLAM_POSE_Handler * handler);

        static MSP_Message serialize_SET_MOTOR(short m1, short m2, short m3, short m4);

    private:

        class ATTITUDE_Handler * handlerForATTITUDE;

        class RC_Handler * handlerForRC;

        class ALTITUDE_Handler * handlerForALTITUDE;

        class SLAM_POSE_Handler * handlerForSLAM_POSE;

};


class ATTITUDE_Handler {

    public:

        ATTITUDE_Handler() {}

        virtual void handle_ATTITUDE(short angx, short angy, short heading){ }

};



class RC_Handler {

    public:

        RC_Handler() {}

        virtual void handle_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8){ }

};



class ALTITUDE_Handler {

    public:

        ALTITUDE_Handler() {}

        virtual void handle_ALTITUDE(int altitude, short vario){ }

};



class SLAM_POSE_Handler {

    public:

        SLAM_POSE_Handler() {}

        virtual void handle_SLAM_POSE(short x, short y, short z, short theta){ }

};

