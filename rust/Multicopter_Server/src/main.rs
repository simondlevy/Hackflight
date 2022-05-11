mod multicopter_server;
use multicopter_server::MulticopterServer;

fn main() {
    let a = MulticopterServer::new(String::from("foo"), 1, 2, 3, 4, 5);
    a.start();
}
