/*
 * Copyright (c) 2023 Simon D. Levy
 *
 * MIT License
 */
#![no_std]

use panic_halt as _; 

#[no_mangle]
pub extern fn add(first: i32, second: i32) -> i32
{
        first + second
}

