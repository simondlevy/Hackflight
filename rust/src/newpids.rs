/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/


pub mod newpids {

    use crate::datatypes::Demands;
    use crate::datatypes::VehicleState;

    #[derive(Debug,Clone)]
    enum PidController {

        AnglePid {
            k_rate_p: f32,
            k_rate_i: f32,
            k_rate_d: f32,
            k_rate_f: f32,
            k_level_p: f32,
            dyn_lpf_previous_quantized_throttle: i32,  
            feedforward_lpf_initialized: bool,
            sum: f32,
        },

        AltitudePid {
            k_p: f32,
            k_i: f32,
            in_band_prev: bool,
            error_integral: f32,
            altitude_target: f32 
        },
    }

    fn get_demands(pid: &PidController) -> Demands {

        match pid {

            PidController::AnglePid {
                k_rate_p,
                k_rate_i,
                k_rate_d,
                k_rate_f,
                k_level_p,
                dyn_lpf_previous_quantized_throttle,  
                feedforward_lpf_initialized,
                sum,
            } => { 
                get_angle_demands(
                    k_rate_p,
                    k_rate_i,
                    k_rate_d,
                    k_rate_f,
                    k_level_p,
                    dyn_lpf_previous_quantized_throttle,  
                    feedforward_lpf_initialized,
                    sum) 
            },

            PidController::AltitudePid {
                k_p,
                k_i,
                in_band_prev,  
                error_integral,
                altitude_target,
            } => { 
                get_alt_hold_demands(
                    k_p,
                    k_i,
                    in_band_prev,  
                    error_integral,
                    altitude_target) 
            },

        }
    }

    fn get_angle_demands(
                    k_rate_p: &f32,
                    k_rate_i: &f32,
                    k_rate_d: &f32,
                    k_rate_f: &f32,
                    k_level_p: &f32,
                    dyn_lpf_previous_quantized_throttle: &i32,  
                    feedforward_lpf_initialized: &bool,
                    sum: &f32) -> Demands  {


        // minimum of 5ms between updates
        const DYN_LPF_THROTTLE_UPDATE_DELAY_US : u16 = 5000; 

        /*
        const uint16_t DYN_LPF_THROTTLE_STEPS = 100;

        // Full iterm suppression in setpoint mode at high-passed setpoint rate
        // > 40deg/sec
        constexpr float ITERM_RELAX_SETPOINT_THRESHOLD = 40;
        const uint8_t   ITERM_RELAX_CUTOFF     = 15;

        const uint16_t DTERM_LPF1_DYN_MIN_HZ = 75;
        const uint16_t DTERM_LPF1_DYN_MAX_HZ = 150;
        const uint16_t DTERM_LPF2_HZ         = 150;

        const uint16_t YAW_LOWPASS_HZ        = 100;

        const uint8_t  ITERM_WINDUP_POINT_PERCENT = 85;        

        const uint8_t D_MIN = 30;
        const uint8_t D_MIN_GAIN = 37;
        const uint8_t D_MIN_ADVANCE = 20;

        const uint8_t FEEDFORWARD_MAX_RATE_LIMIT = 90;

        const uint8_t DYN_LPF_CURVE_EXPO = 5;

        // PT2 lowpass input cutoff to peak D around propwash frequencies
        constexpr float D_MIN_RANGE_HZ   = 85;  

        // PT2 lowpass cutoff to smooth the boost effect
        constexpr float D_MIN_LOWPASS_HZ = 35;  
        constexpr float D_MIN_GAIN_FACTOR          = 0.00008;
        constexpr float D_MIN_SETPOINT_GAIN_FACTOR = 0.00008f;

        const uint16_t RATE_ACCEL_LIMIT = 0;
        const uint16_t YAW_RATE_ACCEL_LIMIT = 0;
        const uint16_t ITERM_LIMIT = 400;

        constexpr float LEVEL_ANGLE_LIMIT = 45;

        constexpr float OUTPUT_SCALING = 1000;
        const uint16_t  LIMIT_YAW  = 400;
        const uint16_t  LIMIT      = 500;
        */

        Demands { 
            throttle : 0.0,
            roll : 0.0,
            pitch : 0.0,
            yaw : 0.0
        }
    }

    fn get_alt_hold_demands(
        k_p: &f32,
        k_i: &f32,
        in_band_prev: &bool,
        error_integral: &f32,
        altitude_target: &f32) -> Demands  {
        Demands { 
            throttle : 0.0,
            roll : 0.0,
            pitch : 0.0,
            yaw : 0.0
        }
    }
}

