/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

// Vehicle state ------------------------------------------------------------------------

struct VehicleState 
{
    x:      f32,
    dx:     f32,
    y:      f32,
    dy:     f32,
    z:      f32,
    dz:     f32,
    phi:    f32,
    dphi:   f32,
    theta:  f32,
    dtheta: f32,
    psi:    f32,
    dpsi:   f32
}


// Scheduling ------------------------------------------------------------------

struct Scheduler
{
    loop_start_cycles: i32,
    loop_start_min_cycles: i32,
    loop_start_max_cycles: i32,
    loop_start_delta_down_cycles: u32,
    loop_start_delta_up_cycles: u32,

    task_guard_cycles: i32,
    task_guard_min_cycles: i32,
    task_guard_max_cycles: i32,
    task_guard_delta_down_cycles: u32,
    task_guard_delta_up_cycles: u32,

    desired_period_cycles: i32,
    last_target_cycles: u32,

    next_timing_cycles: u32,

    guard_margin: i32,
    clock_rate: u32
}


fn main() {
    println!("Hello, world!");
}
