#if 0
//Controller parameters (take note of defaults before modifying!): 
static const float I_LIMIT = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)

static const float MAX_PITCH_ROLL = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode

static const float MAX_YAW = 160.0;     //Max yaw rate in deg/sec

static const float KP_PITCH_ROLL_ANGLE = 0.2;    
static const float KI_PITCH_ROLL_ANGLE = 0.3;    
static const float KD_PITCH_ROLL_ANGLE = 0.05;   

// Damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
static const float B_LOOP_PITCH_ROLL = 0.9;      

static const float KP_PITCH_ROLL_RATE = 0.15;    
static const float KI_PITCH_ROLL_RATE = 0.2;     
static const float KD_PITCH_ROLL_RATE = 0.0002;  

static const float KP_YAW = 0.3;           
static const float KI_YAW = 0.05;          
static const float KD_YAW = 0.00015;       

//IMU calibration parameters
static float ACC_ERROR_X = 0.0;
static float ACC_ERROR_Y = 0.0;
static float ACC_ERROR_Z = 0.0;
static float GYRO_ERROR_X = 0.0;
static float GYRO_ERROR_Y= 0.0;
static float GYRO_ERROR_Z = 0.0;

void controlRATE(const float dt) 
{
    //DESCRIPTION: Computes control commands based on state error (rate)
    /*
     * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
     */
    //Roll
    error_roll = roll_des - GyroX;
    integral_roll = integral_roll_prev + error_roll*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_roll = 0;
    }
    integral_roll = constrain(integral_roll, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_roll = (error_roll - error_roll_prev)/dt; 
    roll_PID = .01*(KP_PITCH_ROLL_RATE*error_roll + KI_PITCH_ROLL_RATE*integral_roll + KD_PITCH_ROLL_RATE*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

    //Pitch
    error_pitch = pitch_des - GyroY;
    integral_pitch = integral_pitch_prev + error_pitch*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_pitch = 0;
    }
    integral_pitch = constrain(integral_pitch, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
    pitch_PID = .01*(KP_PITCH_ROLL_RATE*error_pitch + KI_PITCH_ROLL_RATE*integral_pitch + KD_PITCH_ROLL_RATE*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

    //Yaw, stablize on rate from GyroZ
    error_yaw = yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_yaw = 0;
    }
    integral_yaw = constrain(integral_yaw, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
    yaw_PID = .01*(KP_YAW*error_yaw + KI_YAW*integral_yaw + KD_YAW*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

    //Update roll variables
    error_roll_prev = error_roll;
    integral_roll_prev = integral_roll;
    GyroX_prev = GyroX;
    //Update pitch variables
    error_pitch_prev = error_pitch;
    integral_pitch_prev = integral_pitch;
    GyroY_prev = GyroY;
    //Update yaw variables
    error_yaw_prev = error_yaw;
    integral_yaw_prev = integral_yaw;
}
#endif
