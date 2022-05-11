mod multicopter_server;
use multicopter_server::MulticopterServer;

mod launch_controller;
use launch_controller::LaunchController;

fn main() {
    let a = MulticopterServer::new(String::from("host"), 1, 2, 3, 4, 5);
}
