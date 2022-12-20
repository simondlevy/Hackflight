extern crate hackflight;

use std::net::UdpSocket;

use hackflight::datatypes::Demands;
use hackflight::datatypes::Motors;
use hackflight::datatypes::VehicleState;

#[derive(Debug,Clone)]
enum Shape {
    Square { x: f32, y: f32, s: f32 },
    Circle { x: f32, y: f32, r: f32 },
}

fn area(shape: &Shape) -> f32 {
    match shape {
        Shape::Square { x:_, y:_, s } => {
            s*s
        },
        Shape::Circle { x:_, y:_, r } => {
            std::f32::consts::PI*r*r
        },
    }
}

fn transpose(t: &mut Shape, dx: f32, dy: f32) {

    match *t {

        Shape::Square{ref mut x, ref mut y, s: _} => {*x += dx; *y += dy},
        Shape::Circle{ref mut x, ref mut y, r: _} => {*x += dx; *y += dy},
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

    let circle = Shape::Circle { x: 10.0, y: 20.0, r: 1.0 };
    let square = Shape::Square { x: -5.0, y: 10.0, s: 2.0 };

    let mut shapes: [Shape; 2] = [square, circle];

    loop {

        let mut in_buf = [0; IN_BUF_SIZE]; 
        telemetry_server_socket.recv_from(&mut in_buf)?;

        let time = read_float(in_buf, 0);

        if time < 0.0 { break Ok(()); }

        let vehicle_state = read_vehicle_state(in_buf);

        let mut demands = read_demands(in_buf);

        for shape in shapes.iter_mut() {
            transpose(&mut *shape, 2.0, -3.5);
        }

        let motors = Motors {m1: 0.0, m2: 0.0, m3: 0.0, m4: 0.0};

        let out_buf = write_motors(motors);

        motor_client_socket.send_to(&out_buf, "127.0.0.1:5000")?;
    }
}
