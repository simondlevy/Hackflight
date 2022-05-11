pub struct LaunchController {
    kp: f32,
    ki: f32,
    wind_up_max: f32,
    integral_error: f32,
    tprev: f32
}

impl LaunchController {
    pub fn new (kp: f32, ki: f32) -> LaunchController {
        // First 3 are constants, last 2 are values modified in flight
        LaunchController {kp, ki, wind_up_max:10.0,
                          integral_error:0.0, tprev:0.0}
    }

    pub fn get_demands(mut self, target: f32, alt: f32, vel: f32, t: f32) -> (f32, f32, f32, f32) {
        // Calculate dzdt setpoint and error
        let vel_error = (target - alt) - vel;
        // Compute dt
        let dt = t - self.tprev;

        // Update error integral and error derivative
        self.integral_error += vel_error * dt;
        self.integral_error = constrain_abs(self.integral_error + vel_error * dt, self.wind_up_max);
        // Store time for first difference
        self.tprev = t;
        // Always compute throttle demand for altitude hold
        let throttle = self.kp * vel_error + self.ki * self.integral_error;
        // Don't mess with roll and yaw for now
        let roll = 0.0;
        let yaw = 0.0;
        let mut pitch = 0.0;

        // Pitch forward slightly for one second after level-off
        if 2.0 < t && t < 3.25 {
            pitch = 0.001;
        }

        (throttle, roll, pitch, yaw)
    }
}

fn constrain_abs(x: f32, lim: f32) -> f32 {
    if x < -lim {
        return -lim;
    }
    else {
        if x > lim {
            return lim;
        }
        else {
            return x;
        }
    }
}
