use std::net::{UdpSocket};
use std::error::Error;
use std::{thread, time};

// Initialize constants see Bouabdallah (2004)
pub const STATE_X: u8 = 0;
pub const STATE_DX: u8 = 1;
pub const STATE_Y: u8 = 2;
pub const STATE_DY: u8 = 3;
pub const STATE_Z: u8 = 4;
pub const STATE_DZ: u8 = 5;
pub const STATE_PHI: u8 = 6;
pub const STATE_DPHI: u8 = 7;
pub const STATE_THETA: u8 = 8;
pub const STATE_DTHETA: u8 = 9;
pub const STATE_PSI: u8 = 10;
pub const STATE_DPSI: u8 = 11;

pub struct MulticopterServer {
    host: String, motor_port: u32,
    telemetry_port: u32,
    image_port: u32,
    image_rows: u32,
    image_cols: u32,
    done: bool
}


impl MulticopterServer {
    // Method to initialize it with specified variables
    pub fn new(host: String,
           motor_port: u32,
           telemetry_port: u32,
           image_port: u32,
           image_rows: u32,
           image_cols: u32) -> MulticopterServer {
            MulticopterServer {host, motor_port, telemetry_port,
                               image_port, image_rows, image_cols,
                               done:false}
    }
}

impl MulticopterServer {
    pub fn is_done(self) -> bool {
        self.done
    }

    pub fn start(self) {
        // start the motor_client_socket
        let mut motor_client_addr = self.host.clone();
        motor_client_addr.push_str(":");
        motor_client_addr.push_str(&self.motor_port.to_string());
        let motor_client_socket = UdpSocket::bind(motor_client_addr).unwrap();
        
        //start the telemetry socket
        let mut tele_server_addr = self.host.clone();
        tele_server_addr.push_str(":");
        tele_server_addr.push_str(&self.telemetry_port.to_string());
        let tele_server_socket = UdpSocket::bind(tele_server_addr).unwrap();

        //Start up the thread
        thread::spawn(|| {
            MulticopterServer::run_threadetry(tele_server_socket, motor_client_socket);
        });
        while !self.done {
        }
    }
}

impl MulticopterServer {
    fn run_threadetry(telemetry_server_socket: UdpSocket,
                      motor_client_socket: UdpSocket) -> ! {
        loop {
            let mut buf = [0;17];
            let (values, src) = telemetry_server_socket.recv_from(&mut buf).expect("Crap");
            let telemetry = &mut buf[..values];
            let motor_values = MulticopterServer::get_motors();
            // Convert to bytes to send it over
            for mut item in telemetry.iter() {
                item.to_ne_bytes();
            }
            motor_client_socket.send(telemetry);
            thread::sleep(time::Duration::from_millis(10));
        }
    }
}

impl MulticopterServer {
    pub fn get_motors() -> [f32; 4] {
        [0.6, 0.6, 0.6, 0.6]
    }
}
