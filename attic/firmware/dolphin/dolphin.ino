#include <Wire.h>
#include <Servo.h>

#include <FreeSixIMU.h>
#include <ArduinoPrintf.h>
#include <DueFlashStorage.h>
#include <AttoPilot.h>
#include <TimerTask.h>
#include <ReceiverPoll.h>
#include <dolphinlink.h>

// Constants  ==========================================================================================================

static const int CHANNEL1_PIN = 6;
static const int CHANNEL2_PIN = 5;
static const int CHANNEL3_PIN = 4;
static const int CHANNEL4_PIN = 3;
static const int CHANNEL5_PIN = 2;

static const int LED_PIN = 13;

static const int VOLT_PIN = 0;
static const int AMP_PIN  = 1;


// FrSky VD5M
static const int MIN_PULSE_IN_USEC = 988;
static const int MAX_PULSE_IN_USEC = 2020;

static const int ARM_MARGIN_USEC = 50;

static const int ESC_USEC_MIN = 800;
static const int ESC_USEC_MAX = 2200;

static const int ESC_PINS[4] = {9, 10, 11, 12}; // solder stuck in 8?

// Update rates
static const int IMU_HZ      = 500;
static const int RECEIVER_HZ = 100;
static const int PARAMS_HZ   =   2;

// Maximum allowed settling rate
static const float SETTLE_DEG_MAX = .1;

static const float PITCHROLL_KP_MAX     = 5.0;
static const int   PITCHROLL_KP_PERCENT_DEFAULT = 50;

static const float YAW_KP_MAX           = 5.0;
static const int   YAW_KP_PERCENT_DEFAULT = 50;

static const float PITCHROLL_FACTOR = 0.25;
static const float YAW_FACTOR       = 0.5;
static const float THROTTLE_FACTOR  = 4.0;
static const float THROTTLE_OFFSET  = 2.0;

static const float MAXMOTOR = THROTTLE_FACTOR + THROTTLE_OFFSET + YAW_FACTOR;

// Communications ======================================================================================================

#define XBEE

#ifdef XBEE
#define COMMLINK Serial1
#define COMMBAUD 9600
#else
#define COMMLINK Serial
#define COMMBAUD 9600
#endif

DolphinMessageParser s_parser;

static void sendMessage(DolphinMessage msg) {

    for (byte b=msg.start(); msg.hasNext(); b=msg.getNext()) {

        COMMLINK.write(b);
    }  
}

// Flash storage ======================================================================================================

static void flashWriteFloat(DueFlashStorage & dueFlashStorage, int offset, float value) {

    byte bytes[4];

    memcpy(bytes, &value, 4);

    dueFlashStorage.write(offset,   bytes[0]);
    dueFlashStorage.write(offset+1, bytes[1]);
    dueFlashStorage.write(offset+2, bytes[2]);
    dueFlashStorage.write(offset+3, bytes[3]);
}

static float flashReadFloat(DueFlashStorage & dueFlashStorage, int offset) {

    byte bytes[4];

    bytes[0] = dueFlashStorage.read(offset);
    bytes[1] = dueFlashStorage.read(offset+1);
    bytes[2] = dueFlashStorage.read(offset+2);
    bytes[3] = dueFlashStorage.read(offset+3);

    float value;  
    memcpy(&value, bytes, 4);
    return value;
}

// Arming ==============================================================================================================


static bool s_armed;
static bool s_allow_arming;

static void armingInit() {

    // Arming will be disabled during motors dialog
    s_allow_arming = true;

    // We will toggle an LED to show arming
    pinMode(LED_PIN, OUTPUT);

    s_armed = false;
}

static void sendArmStatus() {

    digitalWrite(LED_PIN, s_armed ? HIGH : LOW);

    sendMessage(s_parser.serialize_ArmStatus(s_armed));
}

class My_ArmEnable_Handler : public ArmEnable_Handler {

    public:

        void handle_ArmEnable(byte enabled) { 

            s_allow_arming = enabled ? true : false;
        }        
};

My_ArmEnable_Handler arm_enable_handler;

// Battery sensing =====================================================================================================

static void sendBatteryStatus() {

    float volts = readVoltage(VOLT_PIN);
    float amps  = readCurrent(AMP_PIN);

    //sendMessage(s_parser.serialize_BatteryStatus( volts, amps));  
}

// R/C input ==========================================================================================================

static bool maxed(short pwm) {

    return pwm > (MAX_PULSE_IN_USEC - ARM_MARGIN_USEC);
}

static bool minned(short pwm) {

    return pwm < (MIN_PULSE_IN_USEC + ARM_MARGIN_USEC);
}

static float s_roll_demand;
static float s_pitch_demand;
static float s_yaw_demand;
static float s_throttle_demand;
static float s_scaled_throttle_demand;

static float scaleDemand(short pwm_in) {

    return ((float)pwm_in - MIN_PULSE_IN_USEC) / (MAX_PULSE_IN_USEC - MIN_PULSE_IN_USEC);
}

static float scaleAxis(short pwm_in) {

    return scaleDemand(pwm_in) *2 - 1;
}

static void processReceiver() {

    short channels[5];

    rxGetChannelValues(channels);

    short throttle = channels[2];
    short yaw = channels[3];

    // throttle down, yaw right to arm
    if (minned(throttle) && maxed(yaw) && s_allow_arming) {
        s_armed = true;
        sendArmStatus();

    }

    // throttle down, yaw left to arm
    if (minned(throttle) && minned(yaw)) {
        s_armed = false;
        sendArmStatus();
    }

    sendMessage(s_parser.serialize_Receiver(channels[0], channels[1], channels[2], channels[3], channels[4]));  

    s_roll_demand  = scaleAxis(channels[0]);
    s_pitch_demand = scaleAxis(channels[1]);
    s_yaw_demand   = scaleAxis(channels[3]);
    s_throttle_demand  = scaleDemand(channels[2]);

    s_scaled_throttle_demand = THROTTLE_FACTOR * sqrt(sqrt(s_throttle_demand)) + THROTTLE_OFFSET;

    s_pitch_demand *= PITCHROLL_FACTOR;
    s_roll_demand  *= PITCHROLL_FACTOR;
    s_yaw_demand   *= YAW_FACTOR;                  
}

// Motors / ESCs =======================================================================================================

static Servo s_escs[4];

static short escScale(short input, short inmin, short inmax) {

    return map(input, inmin, inmax, ESC_USEC_MIN, ESC_USEC_MAX);
}

static void setMotor(short motor, short pwm) {

    s_escs[motor].writeMicroseconds(pwm);
}

class My_Motor_Handler : public Motor_Handler {

    public:

        void handle_Motor(byte motor, byte value) { 

            int pwm = escScale(value, 0, 100);

            // motor indices come in as 1,2,3,4
            setMotor(motor-1, pwm);          
        }        
};

My_Motor_Handler s_motor_handler;

static void initESC(int id) {

    s_escs[id].attach(ESC_PINS[id]);
    s_escs[id].writeMicroseconds(ESC_USEC_MIN);
}

// IMU =================================================================================================================

// From external library
FreeSixIMU s_imu;

// These are flashed from GCS Setup / Calibrate
static float s_roll_baseline_degrees;
static float s_pitch_baseline_degrees;
static int s_pitchroll_kp_percent;
static int s_yaw_kp_percent;
static float s_pitchroll_kp;
static float s_yaw_kp;


static void getFlippedAngles(float * imu_yaw_pitch_roll) {

    s_imu.getYawPitchRoll(imu_yaw_pitch_roll);

    // correct reversed angles: nose-up and roll-right should be positive
    imu_yaw_pitch_roll[1] = -imu_yaw_pitch_roll[1];
    imu_yaw_pitch_roll[2] = -imu_yaw_pitch_roll[2];
}

class My_CalibrateRequest_Handler : public CalibrateRequest_Handler {

    public:

        void handle_CalibrateRequest() { 

            s_roll_baseline_degrees = 0;
            s_pitch_baseline_degrees = 0;          

            // loop till pitch and roll don't change much
            while (true) {
                float imu_yaw_pitch_roll[3];
                getFlippedAngles(imu_yaw_pitch_roll);
                if (s_pitch_baseline_degrees != 0) {
                    if (abs(imu_yaw_pitch_roll[1]-s_pitch_baseline_degrees) < SETTLE_DEG_MAX && 
                            abs(imu_yaw_pitch_roll[2]-s_roll_baseline_degrees)  < SETTLE_DEG_MAX) {
                        break;
                    }
                }

                s_pitch_baseline_degrees = imu_yaw_pitch_roll[1];
                s_roll_baseline_degrees = imu_yaw_pitch_roll[2];
            }

            // tell GCS we're done calibrating
            sendMessage(s_parser.serialize_CalibrateResponse());          
        }        
};

My_CalibrateRequest_Handler s_calibrate_request_handler;

static void processIMU() {

    float imu_yaw_pitch_roll[3];

    getFlippedAngles(imu_yaw_pitch_roll);

    sendMessage(s_parser.serialize_YawPitchRoll(imu_yaw_pitch_roll[0], imu_yaw_pitch_roll[1], imu_yaw_pitch_roll[2]));

    // now send PWM to ESCs

    const float rsign[4] = {+1, -1, -1, +1};
    const float psign[4] = {-1, -1, +1, +1};
    const float ysign[4] = {+1, -1, +1, -1};

    float adjusted_pitch = imu_yaw_pitch_roll[1] - s_pitch_baseline_degrees;
    float adjusted_roll  = imu_yaw_pitch_roll[2] - s_roll_baseline_degrees;

    float pitch_correction =   s_pitchroll_kp * adjusted_pitch / 90;
    float roll_correction  =  -s_pitchroll_kp * adjusted_roll  / 90;

    float yaw = imu_yaw_pitch_roll[0];

    float yaw_correction = 0;
    static float s_yaw_prev = 9999;
    if (s_yaw_prev != 9999) {
        yaw_correction = s_yaw_kp * (yaw - s_yaw_prev);
    }
    s_yaw_prev = yaw;

    arduprintf("%f | ", pitch_correction);

    for (int k=0; k<4; ++k) {

        float motor = s_throttle_demand * 
            (s_scaled_throttle_demand + 
             rsign[k] * (s_roll_demand + roll_correction) +
             psign[k] * (s_pitch_demand + pitch_correction) +
             ysign[k] * (s_yaw_demand + yaw_correction)) / MAXMOTOR;

        short pwm_out = motor * (ESC_USEC_MAX - ESC_USEC_MIN) + ESC_USEC_MIN;

        if (k == 2 || k == 3) {

            pwm_out += 20;
        }

        if (s_armed) {
            setMotor(k, pwm_out);
        }

        arduprintf("%04d ", pwm_out);
    }

    arduprintf("\n");
    Serial.flush();
}

// Params ==============================================================================================================

static void setParams() {

    s_pitchroll_kp =  s_pitchroll_kp_percent / 100. * PITCHROLL_KP_MAX; 
    s_yaw_kp =  s_yaw_kp_percent / 100. * YAW_KP_MAX; 
}

class My_Params_Handler : public Params_Handler {

    public:

        void handle_Params(byte pitchroll_kp_percent, byte yaw_kp_percent) { 

            s_pitchroll_kp_percent = pitchroll_kp_percent;
            s_yaw_kp_percent = yaw_kp_percent;

            setParams();

            DueFlashStorage dueFlashStorage;

            // write sable pitch and roll baselines to flash storage
            flashWriteFloat(dueFlashStorage, 1, s_pitch_baseline_degrees);
            flashWriteFloat(dueFlashStorage, 5, s_roll_baseline_degrees);

            // write PID percentages
            dueFlashStorage.write(9, s_pitchroll_kp_percent);
            dueFlashStorage.write(10, s_yaw_kp_percent);

            // write 0 to address 0 to indicate that it is not the first time running anymore
            dueFlashStorage.write(0, 0); 
        }        
};

My_Params_Handler s_params_handler;


static void processParams() {

    sendMessage(s_parser.serialize_Params(s_pitchroll_kp_percent, s_yaw_kp_percent));
}

// Setup ===============================================================================================================

TimerTask imuTask      = TimerTask(processIMU, IMU_HZ);
TimerTask receiverTask = TimerTask(processReceiver, RECEIVER_HZ);
TimerTask paramsTask   = TimerTask(processParams, PARAMS_HZ);

void setup() {

    Serial.begin(9600);

    COMMLINK.begin(COMMBAUD);

    armingInit();

    // Set up handling of various incoming messages
    s_parser.set_Params_Handler(&s_params_handler);
    s_parser.set_Motor_Handler(&s_motor_handler);
    s_parser.set_ArmEnable_Handler(&arm_enable_handler);
    s_parser.set_CalibrateRequest_Handler(&s_calibrate_request_handler);

    s_imu.init();  

    // Init motors  
    initESC(0);
    initESC(1);
    initESC(2);
    initESC(3);

    // Init receiver channels
    rxInitChannels(CHANNEL1_PIN, CHANNEL2_PIN, CHANNEL3_PIN, CHANNEL4_PIN, CHANNEL5_PIN);

    // Grab pre-flashed settings
    DueFlashStorage dueFlashStorage;
    s_pitch_baseline_degrees  = flashReadFloat(dueFlashStorage, 1); 
    s_roll_baseline_degrees   = flashReadFloat(dueFlashStorage, 5); 
    s_pitchroll_kp_percent = dueFlashStorage.read(9); 
    s_yaw_kp_percent = dueFlashStorage.read(10); 

    // Set bad values to defaults
    if (s_roll_baseline_degrees != s_roll_baseline_degrees) {

        s_roll_baseline_degrees = 0;
        s_pitch_baseline_degrees = 0;
        s_pitchroll_kp_percent = PITCHROLL_KP_PERCENT_DEFAULT;
        s_yaw_kp_percent = YAW_KP_PERCENT_DEFAULT;
    }  

    // Initialize params
    setParams();
}

// Loop ================================================================================================================


void loop() {

    // Update timer tasks
    imuTask.update();
    receiverTask.update();
    paramsTask.update();

    // Parse incoming messages if available
    if (COMMLINK.available()) {

        // disallow arming when GCS is in control
        s_armed = false;

        s_parser.parse(COMMLINK.read());
    }

}
