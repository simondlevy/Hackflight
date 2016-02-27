static const uint8_t CONFIG_LEVEL_P  = 90;
static const uint8_t CONFIG_LEVEL_I  = 10;
static const uint8_t CONFIG_LEVEL_D  = 100;

// roll, pitch, yaw
static const uint8_t CONFIG_AXIS_P[3] = {40, 40, 85};
static const uint8_t CONFIG_AXIS_I[3] = {30, 30, 45};
static const uint8_t CONFIG_AXIS_D[3] = {23, 23, 0};

// altitude
static const uint8_t CONFIG_ALT_P = 50;
static const uint8_t CONFIG_VEL_P = 120;
static const uint8_t CONFIG_VEL_I = 45;
static const uint8_t CONFIG_VEL_D = 1;

// PWM
static const bool     CONFIG_USE_CPPM            = true;
static const bool     CONFIG_PWM_FILTER          = false;
static const bool     CONFIG_FAST_PWM            = false;
static const uint32_t CONFIG_MOTOR_PWM_RATE      = 400;
static const uint16_t CONFIG_PWM_IDLE_PULSE_USEC = 1000; // standard PWM for brushless ESC 

static const uint8_t  CONFIG_ROLL_PITCH_RATE[2] = {0, 0};
static const int16_t  CONFIG_ANGLE_TRIM[2]      = {0, 0};
static const int      CONFIG_RCMAP[8]           = {0, 1, 3, 2, 4, 5, 6, 7};

static const uint32_t CONFIG_ALT_UPDATE_USEC           = 25000; // 40hz update rate (20hz LPF on acc)
static const bool     CONFIG_HORIZON_MODE              = true; 
static const uint16_t CONFIG_CALIBRATING_GYRO_CYCLES   = 1000;
static const uint16_t CONFIG_CALIBRATING_ACC_CYCLES    =  400;
static const int16_t  CONFIG_MIDRC                     = 1500;
static const int16_t  CONFIG_MINCOMMAND                = 1000;
static const uint16_t CONFIG_GYRO_CMPF_FACTOR          = 600;
static const uint16_t CONFIG_MINTHROTTLE               = 990;
static const uint16_t CONFIG_MAXTHROTTLE               = 2010;
static const uint16_t CONFIG_MINCHECK                  = 1100;
static const int8_t   CONFIG_YAW_CONTROL_DIRECTION     = 1;   // 1 or -1
static const uint16_t CONFIG_MAX_ANGLE_INCLINATION     = 500;   // 50 degrees
static const uint16_t CONFIG_MAXCHECK                  = 1900;
static const uint8_t  CONFIG_ALT_HOLD_THROTTLE_NEUTRAL = 40;
static const bool     CONFIG_ALT_HOLD_FAST_CHANGE      = true;
static const uint16_t CONFIG_IMU_LOOPTIME_USEC         = 3500;
static const uint32_t CONFIG_RC_LOOPTIME_USEC          = 20000;
static const uint32_t CONFIG_CALIBRATE_ACCTIME_USEC    = 500000;
static const uint8_t  CONFIG_BARO_TAB_SIZE             = 21;
static const float    CONFIG_BARO_NOISE_LPF            = 0.6;
static const float    CONFIG_BARO_CF_ALT               = 0.965;
static const float    CONFIG_BARO_CF_VEL               = 0.985;
static const uint8_t  CONFIG_MORON_THRESHOLD           = 32;
static const char     CONFIG_REBOOT_CHARACTER          = 'R';
static const uint8_t  CONFIG_RC_EXPO_8                 = 65;
static const uint8_t  CONFIG_RC_RATE_8                 = 90;
static const uint8_t  CONFIG_THR_MID_8                 = 50;
static const uint8_t  CONFIG_THR_EXPO_8                = 0;
static const int8_t   CONFIG_YAW_DIRECTION             = 1;

static const sensor_align_e CONFIG_ACC_ALIGN           = ALIGN_DEFAULT;
static const sensor_align_e CONFIG_GYRO_ALIGN          = ALIGN_DEFAULT;

// the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
static const uint16_t CONFIG_THROTTLE_CORRECTION_ANGLE =       800;

// the correction that will be applied at throttle_correction_angle
static const uint8_t CONFIG_THROTTLE_CORRECTION_VALUE  = 0;

static const uint8_t  CONFIG_YAW_RATE                  = 0;
static const uint16_t CONFIG_GYRO_LPF                  = 42;
static const uint8_t  CONFIG_DYN_THR_PID               = 0;
static const uint16_t CONFIG_TPA_BREAKPOINT            = 1500;
static const uint8_t  CONFIG_ACC_LPF_FACTOR            = 4;
static const uint8_t  CONFIG_ACCZ_DEADBAND             = 40;
static const uint8_t  CONFIG_ACCXY_DEADBAND            = 40;
static const float    CONFIG_ACCZ_LPF_CUTOFF           = 5.0;
static const bool     CONFIG_ACC_UNARMEDCAL            = true;
static const uint8_t  CONFIG_SMALL_ANGLE               = 25;
static const bool     CONFIG_DEADBAND                  = false;
static const bool     CONFIG_YAW_DEADBAND              = false;


