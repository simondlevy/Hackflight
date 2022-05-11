pub struct LaunchController {
    Kp: f32,
    Ki: f32,
    wind_up_max: f32,
    integral_error: f32,
    tprev: f32
}

impl LaunchController {
    pub fn new (Kp: f32, Ki: f32) -> LaunchController {
        LaunchController {Kp, Ki, wind_up_max:10.0,
                          integral_error:0.0, tprev:0.0}
    }

    pub fn get_demands(mut self, target: f32, alt: f32, vel: f32, t: f32) -> (f32, f32, f32, f32) {
        let vel_error = (target - alt) - vel;
        let dt = t - self.tprev;

        self.integral_error += vel_error * dt;
        self.integral_error = constrain_abs(self.integral_error + vel_error * dt, self.wind_up_max);
        self.tprev = t;
        let throttle = self.Kp * vel_error + self.Ki * self.integral_error;
        let roll = 0.0;
        let yaw = 0.0;
        let mut pitch = 0.0;

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
