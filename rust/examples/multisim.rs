extern crate hackflight;

use std::net::UdpSocket;

use hackflight::Demands;
use hackflight::Motors;
use hackflight::VehicleState;
use hackflight::pids;
use hackflight::step;
use hackflight::mixers::quadxbf;
use hackflight::utils::rescale;
use hackflight::utils::rad2deg;

const RATE_KP  : f32 = 1.441305;
const RATE_KI  : f32 = 48.8762;
const RATE_KD  : f32 = 0.021160;
const RATE_KF  : f32 = 0.0165048;
const LEVEL_KP : f32 = 0.0;

const ALT_HOLD_KP : f32 = 7.5e-2;
const ALT_HOLD_KI : f32 = 1.5e-1;

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

    fn read_degrees(buf:[u8; IN_BUF_SIZE], idx:usize) -> f32 {
        rad2deg(read_float(buf, idx))
    }

    fn state_from_telemetry(buf:[u8; IN_BUF_SIZE]) -> VehicleState {
        VehicleState {
            x:read_float(buf, 1),       
            dx:read_float(buf, 2),
            y:read_float(buf, 3),
            dy:read_float(buf, 4),
            z:-read_float(buf, 5),         // NED => ENU
            dz:-read_float(buf, 6),        // NED => ENU
            phi:read_degrees(buf, 7),
            dphi:read_degrees(buf, 8),
            theta:-read_degrees(buf, 9),   // note sign reversal
            dtheta:-read_degrees(buf, 10), // note sign reversal
            psi:read_degrees(buf, 11),
            dpsi:read_degrees(buf, 12)
        }
    }

    fn demands_from_telemetry(buf:[u8; IN_BUF_SIZE]) -> Demands {
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

    let alt_hold_pid = pids::make_alt_hold(ALT_HOLD_KP, ALT_HOLD_KI);

    let angle_pid = pids::make_angle(RATE_KP, RATE_KI, RATE_KD, RATE_KF, LEVEL_KP);

    let mixer = quadxbf::QuadXbf { };

    let mut pids: [pids::Controller; 2] = [angle_pid, alt_hold_pid];

    // Loop forever, waiting for client
    loop {

        // Get incoming telemetry values
        let mut in_buf = [0; IN_BUF_SIZE]; 
        telemetry_server_socket.recv_from(&mut in_buf)?;

        // Sim sends negative time value on halt
        let time = read_float(in_buf, 0);
        if time < 0.0 { 
            break Ok(()); 
        }

        // Convert simulator time to microseconds
        let usec = (time * 1e6) as u32;

        // Build vehicle state 
        let vstate = state_from_telemetry(in_buf);

        // Get incoming stick demands
        let mut stick_demands = demands_from_telemetry(in_buf);

        // Reset PID controllers on zero throttle
        let pid_reset = stick_demands.throttle < 0.05;

        // Rescale throttle [-1,+1] => [0,1]
        stick_demands.throttle = rescale(stick_demands.throttle, -1.0, 1.0, 0.0, 1.0);

        // let motors = Motors {m1: 0.0, m2: 0.0, m3:0.0, m4:0.0};
        let motors = step(&stick_demands, &vstate, &mut pids, &pid_reset, &usec, &mixer);

        let out_buf = write_motors(motors);

        motor_client_socket.send_to(&out_buf, "127.0.0.1:5000")?;
    }
}
