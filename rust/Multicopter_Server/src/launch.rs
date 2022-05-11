mod multicopter_server;
use multicopter_server::MulticopterServer;

mod LaunchController;
use LaunchController::LaunchController;

struct LaunchCopter {
    mixer: Mixer,
    kp: f32,
    ki: f32,
    initial_target: f32,
    time: f32,
    ctrl: LaunchController
};

impl LaunchCopter {
    pub fn new (mixer: Mixer) -> LaunchCopter {
        LaunchCopter{mixer, kp=1.0, ki=0.0, initial_target=15.0,
                     time=0.0, ctrl=LaunchController::new{self.kp, self.ki}}
    }

    // TODO when I figure out how to do images at all
    pub fn handle_image(){}
    pub fn get_motors (self, t: f32, state
}
