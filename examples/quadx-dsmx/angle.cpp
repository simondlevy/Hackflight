#include <hackflight.hpp>
#include <utils.hpp>

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

void controlANGLE(
        const float dt, 
        const float roll_des, 
        const float pitch_des, 
        const float yaw_des, 
        const float roll_IMU,
        const float pitch_IMU,
        const uint32_t channel_1_pwm,
        const float GyroX,
        const float GyroY,
        const float GyroZ,
        float & roll_PID,
        float & pitch_PID,
        float & yaw_PID) 
{
    //DESCRIPTION: Computes control commands based on state error (angle)
    /*
     * Basic PID control to stablize on angle setpoint based on desired states
     * roll_des, pitch_des, and yaw_des computed in getDesState(). Error is
     * simply the desired state minus the actual state (ex. roll_des -
     * roll_IMU). Two safety features are implimented here regarding the I
     * terms. The I terms are saturated within specified limits on startup to
     * prevent excessive buildup. This can be seen by holding the vehicle at an
     * angle and seeing the motors ramp up on one side until they've maxed out
     * throttle...saturating I to a specified limit fixes this. The second
     * feature defaults the I terms to 0 if the throttle is at the minimum
     * setting. This means the motors will not start spooling up on the ground,
     * and the I terms will always start from 0 on takeoff. This function
     * updates the variables roll_PID, pitch_PID, and yaw_PID which can be
     * thought of as 1-D stablized signals. They are mixed to the configuration
     * of the vehicle in the mixer.
     */

    static float _integral_roll;
    static float _integral_roll_prev;

    static float _integral_pitch;
    static float _integral_pitch_prev;

    static float _integral_yaw;
    static float _integral_yaw_prev;

    static float _error_yaw_prev;

    //Roll
    const auto error_roll = roll_des - roll_IMU;
    _integral_roll = _integral_roll_prev + error_roll*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        _integral_roll = 0;
    }
    _integral_roll = hf::Utils::fconstrain(_integral_roll, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    const auto derivative_roll = GyroX;
    roll_PID = 0.01*(KP_PITCH_ROLL_ANGLE*error_roll + KI_PITCH_ROLL_ANGLE*_integral_roll - KD_PITCH_ROLL_ANGLE*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

    //Pitch
    const auto error_pitch = pitch_des - pitch_IMU;
    _integral_pitch = _integral_pitch_prev + error_pitch*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        _integral_pitch = 0;
    }
    _integral_pitch = hf::Utils::fconstrain(_integral_pitch, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    const auto derivative_pitch = GyroY;
    pitch_PID = .01*(KP_PITCH_ROLL_ANGLE*error_pitch + KI_PITCH_ROLL_ANGLE*_integral_pitch - KD_PITCH_ROLL_ANGLE*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

    //Yaw, stablize on rate from GyroZ
    const auto error_yaw = yaw_des - GyroZ;
    _integral_yaw = _integral_yaw_prev + error_yaw*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        _integral_yaw = 0;
    }
    _integral_yaw = hf::Utils::fconstrain(_integral_yaw, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    const auto derivative_yaw = (error_yaw - _error_yaw_prev)/dt; 
    yaw_PID = .01*(KP_YAW*error_yaw + KI_YAW*_integral_yaw + KD_YAW*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

    //Update roll variables
    _integral_roll_prev = _integral_roll;
    //Update pitch variables
    _integral_pitch_prev = _integral_pitch;
    //Update yaw variables
    _error_yaw_prev = error_yaw;
    _integral_yaw_prev = _integral_yaw;
}
