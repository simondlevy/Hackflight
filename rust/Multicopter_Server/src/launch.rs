use crate::multicopter_server::{MulticopterServer, STATE_Z, STATE_DZ};

use crate::launch_controller::LaunchController;

use crate::mixers::Mixer;

pub struct LaunchCopter {
    mixer: Mixer,
    kp: f32,
    ki: f32,
    initial_target: f32,
    time: f32,
    ctrl: LaunchController
}

impl LaunchCopter {
    pub fn new (&self, mixer: Mixer) -> LaunchCopter {
        LaunchCopter{mixer, kp:1.0, ki:0.0, initial_target:15.0,
                     time:0.0, ctrl:LaunchController::new(self.kp, self.ki)}
    }

    // TODO when I figure out how to do images at all
    pub fn handle_image(){}

    pub fn get_motors (mut self, t: f32,
                       state: (f32, f32, f32, f32, f32, f32, f32, f32, f32, f32, f32, f32)) {
        self.time = t;

        // Extract altitude from stat. Altitude is in NED coordinates, so we
        // negate it to use as input to PID controller.
        // TODO Replace magic numbers with STATE_Z and STATE_DZ
        let z = -state.4;
        let dzdt = -state.5;

        // Get demands U [throttle, roll, pitch, yaw] from PID controller
        let u = self.ctrl.get_demands(self.initial_target, z, dzdt, t);

        let omega = self.mixer.get_motors(u);

        // Constrain motor values to [0, 1]
    }
}
