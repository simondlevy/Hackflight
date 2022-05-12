mod multicopter_server;
use multicopter_server::MulticopterServer;

mod launch_controller;
use launch_controller::LaunchController;

mod launch;
use launch::LaunchCopter;

mod mixers;
use mixers::Mixer;

/*
A Tester main for testing and so that i don't get yelled at by cargo
*/
fn main() {
    let phantom_mixer = Mixer::new_phantom_mixer();
    let copter = LaunchCopter::new(phantom_mixer);
}
