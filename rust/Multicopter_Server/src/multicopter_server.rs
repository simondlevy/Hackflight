use std::net::{UdpSocket};

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
    host: String,
    motor_port: u32,
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
    pub fn get_host(self) -> String {
        self.host
    }

    pub fn start(self) {
        // start the motor_client_socket
        let mut motor_client_addr = self.host.clone();
        motor_client_addr.push_str(":");
        motor_client_addr.push_str(&self.motor_port.to_string());
        let motor_client_socket = UdpSocket::bind(motor_client_addr);
        
        //start the telemetry socket
        let mut tele_server_addr = self.host.clone();
        tele_server_addr.push_str(":");
        tele_server_addr.push_str(&self.telemetry_port.to_string());
        let tele_server_socket = UdpSocket::bind(tele_server_addr);
        while !self.done {
        }
    }
}
