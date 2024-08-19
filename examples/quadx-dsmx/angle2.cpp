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

void controlANGLE2(const float dt) 
{
    //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
    /*
     * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
     * See the documentation for tuning this controller.
     */
    //Outer loop - PID on angle
    float roll_des_ol, pitch_des_ol;
    //Roll
    error_roll = roll_des - roll_IMU;
    integral_roll_ol = integral_roll_prev_ol + error_roll*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_roll_ol = 0;
    }
    integral_roll_ol = constrain(integral_roll_ol, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_roll = (roll_IMU - roll_IMU_prev)/dt; 
    roll_des_ol = KP_PITCH_ROLL_ANGLE*error_roll + KI_PITCH_ROLL_ANGLE*integral_roll_ol;// - KD_PITCH_ROLL_ANGLE*derivative_roll;

    //Pitch
    error_pitch = pitch_des - pitch_IMU;
    integral_pitch_ol = integral_pitch_prev_ol + error_pitch*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_pitch_ol = 0;
    }
    integral_pitch_ol = constrain(integral_pitch_ol, -I_LIMIT, I_LIMIT); //saturate integrator to prevent unsafe buildup
    derivative_pitch = (pitch_IMU - pitch_IMU_prev)/dt;
    pitch_des_ol = KP_PITCH_ROLL_ANGLE*error_pitch + KI_PITCH_ROLL_ANGLE*integral_pitch_ol;// - KD_PITCH_ROLL_ANGLE*derivative_pitch;

    //Apply loop gain, constrain, and LP filter for artificial damping
    float Kl = 30.0;
    roll_des_ol = Kl*roll_des_ol;
    pitch_des_ol = Kl*pitch_des_ol;
    roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
    pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
    roll_des_ol = (1.0 - B_LOOP_PITCH_ROLL)*roll_des_prev + B_LOOP_PITCH_ROLL*roll_des_ol;
    pitch_des_ol = (1.0 - B_LOOP_PITCH_ROLL)*pitch_des_prev + B_LOOP_PITCH_ROLL*pitch_des_ol;

    //Inner loop - PID on rate
    //Roll
    error_roll = roll_des_ol - GyroX;
    integral_roll_il = integral_roll_prev_il + error_roll*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_roll_il = 0;
    }
    integral_roll_il = constrain(integral_roll_il, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_roll = (error_roll - error_roll_prev)/dt; 
    roll_PID = .01*(KP_PITCH_ROLL_RATE*error_roll + KI_PITCH_ROLL_RATE*integral_roll_il + KD_PITCH_ROLL_RATE*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

    //Pitch
    error_pitch = pitch_des_ol - GyroY;
    integral_pitch_il = integral_pitch_prev_il + error_pitch*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_pitch_il = 0;
    }
    integral_pitch_il = constrain(integral_pitch_il, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
    pitch_PID = .01*(KP_PITCH_ROLL_RATE*error_pitch + KI_PITCH_ROLL_RATE*integral_pitch_il + KD_PITCH_ROLL_RATE*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

    //Yaw
    error_yaw = yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_yaw = 0;
    }
    integral_yaw = constrain(integral_yaw, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
    yaw_PID = .01*(KP_YAW*error_yaw + KI_YAW*integral_yaw + KD_YAW*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

    //Update roll variables
    integral_roll_prev_ol = integral_roll_ol;
    integral_roll_prev_il = integral_roll_il;
    error_roll_prev = error_roll;
    roll_IMU_prev = roll_IMU;
    roll_des_prev = roll_des_ol;
    //Update pitch variables
    integral_pitch_prev_ol = integral_pitch_ol;
    integral_pitch_prev_il = integral_pitch_il;
    error_pitch_prev = error_pitch;
    pitch_IMU_prev = pitch_IMU;
    pitch_des_prev = pitch_des_ol;
    //Update yaw variables
    error_yaw_prev = error_yaw;
    integral_yaw_prev = integral_yaw;

}
#endif
