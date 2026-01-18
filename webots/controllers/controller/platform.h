/* 
   C++ flight simulator API

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

void platform_init();
bool platform_step();
const char * platform_joystick_get_model();
void platform_cleanup();
double platform_get_vehicle_x();
double platform_get_vehicle_y();
double platform_get_vehicle_z();
double platform_get_vehicle_phi();
double platform_get_vehicle_theta();
double platform_get_vehicle_psi();
void platform_send_siminfo(const void * info, const size_t size);
int platform_joystick_get_axis_value(const uint8_t axis);
int platform_joystick_get_pressed_button();
const char * platform_joystick_get_name();
int platform_joystick_get_number_of_axes();
int platform_keyboard_get_key();
float platform_get_framerate();
int platform_keyboard_down();
int platform_keyboard_left();
int platform_keyboard_right();
int platform_keyboard_up();
