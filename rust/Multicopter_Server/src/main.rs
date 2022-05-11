mod multicopter_server;
use multicopter_server::MulticopterServer;

mod launch_controller;
use launch_controller::LaunchController;

mod mixers;
use mixers::Mixer;

/*
A Tester main for testing and so that i don't get yelled at by cargo
*/
fn main() {
    let a = MulticopterServer::new(String::from("host"), 1, 2, 3, 4, 5);
    let b = LaunchController::new(0.0, 0.0);
}
