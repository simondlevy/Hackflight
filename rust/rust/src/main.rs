/*
   Hackflight-style server for MulticopterSim socket module

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

use std::net::UdpSocket;

use datatypes::datatypes::Demands;
use datatypes::datatypes::Motors;
use datatypes::datatypes::VehicleState;

use pids::altitude as altitude_pid;
use pids::yaw as yaw_pid;
use pids::pids::new_controller as new_pid_controller;
use hackflight::hackflight::run_hackflight;
use mixers::fixedpitch::quadxbf as mixer;

pub mod datatypes;
pub mod hackflight;
pub mod mixers;
pub mod pids;
pub mod utils;

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

    let mut altitude_pid = altitude_pid::new();
    let mut yaw_pid = yaw_pid::new();

    let mut pid_controller = new_pid_controller();

    println!("Hit the Play button ...");

    loop {

        let mut in_buf = [0; IN_BUF_SIZE]; 
        telemetry_server_socket.recv_from(&mut in_buf)?;

        let time = read_float(in_buf, 0);

        if time < 0.0 { break Ok(()); }

        let vehicle_state = read_vehicle_state(in_buf);

        let demands = read_demands(in_buf);

        let (motors, new_altitude_pid, new_yaw_pid) =
            run_hackflight(
                demands,
                vehicle_state,
                altitude_pid.clone(),
                yaw_pid.clone(),
                &mixer::run
                );

        altitude_pid = new_altitude_pid;
        yaw_pid = new_yaw_pid;

        let out_buf = write_motors(motors);

        motor_client_socket.send_to(&out_buf, "127.0.0.1:5000")?;
    }
}
