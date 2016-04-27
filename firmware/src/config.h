static const uint8_t CONFIG_LEVEL_P  = 90;
static const uint8_t CONFIG_LEVEL_I  = 10;
static const uint8_t CONFIG_LEVEL_D  = 100;

// roll, pitch, yaw
static const uint8_t CONFIG_AXIS_P[3] = {40, 40, 85};
static const uint8_t CONFIG_AXIS_I[3] = {30, 30, 45};
static const uint8_t CONFIG_AXIS_D[3] = {23, 23, 0};

#define CONFIG_MAGNETIC_DECLINATION                 0
#define CONFIG_HORIZON_MODE                         true
#define CONFIG_CALIBRATING_GYRO_CYCLES              1000
#define CONFIG_CALIBRATING_ACC_CYCLES               400
#define CONFIG_MIDRC                                1500
#define CONFIG_MINCOMMAND                           1000
#define CONFIG_GYRO_CMPF_FACTOR                     600    
#define CONFIG_GYRO_CMPFM_FACTOR                    250  
#define CONFIG_MINTHROTTLE                          990
#define CONFIG_MAXTHROTTLE                          2010
#define CONFIG_MINCHECK                             1100
#define CONFIG_YAW_CONTROL_DIRECTION                1   /* 1 or -1 */
#define CONFIG_MAX_ANGLE_INCLINATION                500 /* 50 degrees */
#define CONFIG_MAXCHECK                             1900

#define CONFIG_IMU_LOOPTIME_USEC                    3500
#define CONFIG_RC_LOOPTIME_USEC                     20000
#define CONFIG_CALIBRATE_ACCTIME_USEC               500000

#define CONFIG_MORON_THRESHOLD                      32
#define CONFIG_REBOOT_CHARACTER                     'R'
#define CONFIG_RC_EXPO_8                            65
#define CONFIG_RC_RATE_8                            90
#define CONFIG_THR_MID_8                            50
#define CONFIG_THR_EXPO_8                           0
#define CONFIG_YAW_DIRECTION                        1
#define CONFIG_FAILSAFE_DELAY                       10   /* 1sec */

// the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
#define CONFIG_THROTTLE_CORRECTION_ANGLE            800   

// the correction that will be applied at throttle_correction_angle.
#define CONFIG_THROTTLE_CORRECTION_VALUE            0
#define CONFIG_YAW_RATE                             0

#define CONFIG_DYN_THR_PID                          0
#define CONFIG_TPA_BREAKPOINT                       1500
#define CONFIG_ACC_LPF_FACTOR                       4
#define CONFIG_ACCZ_DEADBAND                        40
#define CONFIG_ACCXY_DEADBAND                       40
#define CONFIG_ACCZ_LPF_CUTOFF                      5.0F
#define CONFIG_ACC_UNARMEDCAL                       1
#define CONFIG_SMALL_ANGLE                          25
#define CONFIG_DEADBAND                             0
#define CONFIG_YAW_DEADBAND                         0
#define CONFIG_BARO_TAB_SIZE                        21

static const uint8_t CONFIG_ROLL_PITCH_RATE[2] = {0, 0};
static const int16_t CONFIG_ANGLE_TRIM[2]      = {0, 0};
static const uint8_t CONFIG_RCMAP[8]           = {0, 1, 3, 2, 4, 5, 6, 7};
