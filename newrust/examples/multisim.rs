extern crate hackflight;

use std::net::UdpSocket;

use hackflight::datatypes::Demands;
use hackflight::datatypes::Motors;
use hackflight::datatypes::VehicleState;

#[derive(Debug,Clone)]
enum PidController {

    AnglePid { 
        x: f32,
        y: f32,
        s: f32 
    },

    AltHoldPid { 
        k_p : f32,
        k_i: f32, 
        in_band_prev: bool,
        error_integral: f32,
        altitude_target: f32
    },
}

fn get_demands(t: &mut PidController, dx: f32, dy: f32) {

    match *t {

        PidController::AnglePid {
            ref mut x,
            ref mut y,
            s: _
        } => {

            *x += dx;
            *y += dy
        },

        PidController::AltHoldPid {
            ref mut k_p ,
            ref mut k_i, 
            ref mut in_band_prev,
            ref mut error_integral,
            ref mut altitude_target
        } => {

            *in_band_prev = false
        },
    }
}

fn main() -> std::io::Result<()> {

    const IN_BUF_SIZE:usize  = 17*8; // 17 doubles in
    const OUT_BUF_SIZE:usize = 4*8;  // 4 doubles out

    fn read_float(buf:[u8; IN_BUF_SIZE], idx:usize) -> f32 {
        let mut dst = [0u8; 8];
        let beg = 8 * idx;
        let end = beg + 8;
        dst.clone_from_slice(&buf[beg..end]);
        f64::from_le_bytes(dst) as f32
    }

    fn read_vehicle_state(buf:[u8; IN_BUF_SIZE]) -> VehicleState {
        VehicleState {
            x:read_float(buf, 1),
            dx:read_float(buf, 2),
            y:read_float(buf, 3),
            dy:read_float(buf, 4),
            z:read_float(buf, 5),
            dz:read_float(buf, 6),
            phi:read_float(buf, 7),
            dphi:read_float(buf, 8),
            theta:read_float(buf, 9),
            dtheta:read_float(buf, 10),
            psi:read_float(buf, 11),
            dpsi:read_float(buf, 12)
        }
    }

    fn read_demands(buf:[u8; IN_BUF_SIZE]) -> Demands {
        Demands {
            throttle:read_float(buf, 13),
            roll:read_float(buf, 14),
            pitch:read_float(buf, 15),
            yaw:read_float(buf, 16)
        }
    }

    fn write_motors(motors:Motors) -> [u8; OUT_BUF_SIZE] {
        let mut buf = [0u8; OUT_BUF_SIZE]; 
        let motorvals = [motors.m1, motors.m2, motors.m3, motors.m4];
        for j in 0..4 {
            let bytes = (motorvals[j] as f64).to_le_bytes();
            for k in 0..8 {
                buf[j*8+k] = bytes[k];
            }
        }
        buf
    }

    // We have to bind client socket to some address
    let motor_client_socket = UdpSocket::bind("0.0.0.0:0")?;

    // Bind server socket to address,port that client will connect to
    let telemetry_server_socket = UdpSocket::bind("127.0.0.1:5001")?;

    println!("Hit the Play button ...");

    let alt_hold_pid = PidController::AltHoldPid {
        k_p : 0.0,
        k_i: 0.0, 
        in_band_prev: false,
        error_integral: 0.0,
        altitude_target: 0.0
     };

    let angle_pid = PidController::AnglePid { x: -5.0, y: 10.0, s: 2.0 };

    let mut pids: [PidController; 2] = [angle_pid, alt_hold_pid];

    loop {

        let mut in_buf = [0; IN_BUF_SIZE]; 
        telemetry_server_socket.recv_from(&mut in_buf)?;

        let time = read_float(in_buf, 0);

        if time < 0.0 { break Ok(()); }

        let vehicle_state = read_vehicle_state(in_buf);

        let mut demands = read_demands(in_buf);

        for shape in pids.iter_mut() {
            get_demands(&mut *shape, 2.0, -3.5);
        }

        let motors = Motors {m1: 0.0, m2: 0.0, m3: 0.0, m4: 0.0};

        let out_buf = write_motors(motors);

        motor_client_socket.send_to(&out_buf, "127.0.0.1:5000")?;
    }
}
