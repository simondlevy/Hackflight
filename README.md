# Hackflight: Simple quadcopter flight control firmware and simulator for C++ hackers

Hackflight is simple header-only C++ firmware for quadcopter flight
controllers and simulators.  It is geared toward people like me who want to
tinker with flight-control firmware, and use it to teach students about ideas
like inertial measurement and PID tuning.  <b>If you are in the 99% percent of
users who just want to get your vehicle flying without getting into firmware
hacking, I recommend [Cleanflight](http://cleanflight.com/)</b> (great for
getting started when you're on a budget) <b>or the
[Ardupilot](http://copter.ardupilot.org/ardupilot/index.html) system</b> (for
sophisticated mission planning with waypoint navigation and the like).  In
addition to big user communities and loads of great features, these platforms
have safety mechanisms that Hackflight lacks, which will help avoid injury to
you and damage to your vehicle.

Hackflight derives from the Baseflight firmware (which in turn derives from
Multiwii), and currently works on STM32F103 flight-controller boards
(Naze32 and clones like Flip32, MultiRC, etc.), the Alienflight F3 board,
and the Arduino-compatible Teensy 3.1/3.2
microcontroller (with additional hardware).  Thanks to a major effort by
[Sytelus](https://github.com/sytelus), the core Hackflight 
[firmware ](https://github.com/simondlevy/hackflight/tree/master/firmware) now
adheres to best practices for C++.  As you can 
[see](https://github.com/simondlevy/hackflight/blob/master/boards/alienflightf3/hackflight.cpp), 
the code follows the Arduino
design pattern of a <tt>startup</tt> routine that calls the
<tt>init()</tt> method of a few objects (<tt>IMU</tt>, <tt>RC</tt>,
<tt>PID</tt>, <tt>Board</tt>) and a <tt>loop</tt> routine that calls the
<tt>update()</tt> method and other methods of those objects.  The code provides
abstraction (through the <tt>Board</tt> class) that should make it easy to use
on other boards.  

The only parameters you should need to adjust are the PID tuning 
[params](https://github.com/simondlevy/hackflight/blob/master/firmware/config.hpp#L25-43).  As 
with Baseflight, you get a gyro auto-calibration sequence on startup, indicated
by  steady green LED that turns off when the calibration is done.  You can
re-calibrate the gyro by putting the collective (left) stick in full upper-left
and the cyclic (right) in full center-down position.  You can calibrate the
accelerometer with collective lower-left and cyclic center-down.  As usual,
collective lower-right arms the board, and lower-left disarms it, as indicated
by the red LED.  The green LED will flash when the board is tilted by more than
25 degrees.

Although Hackflight was designed to be &ldquo;headless&rdquo; (no configurator program),
it is useful to get some visual feedback on things like vehicle orientation and RC receiver
PWM values.  So in the <tt>gcs</tt> folder you'll find a Python program (<tt>main.py</tt>)
that allows you to connect to the board and see what's going on.  To use this program you'll
need to install [MSPPG](https://github.com/simondlevy/hackflight/tree/master/parser), a
parser generator for the Multiwii Serial Protocol (MSP) messages used by the
firmware. Follow the directions in that repository to install MSPPG for Python.

If you find Hackflight useful, please consider donating
to the [Baseflight](https://goo.gl/3tyFhz) or 
[Cleanflight](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=TSQKVT6UYKGL6)
projects from which it is derived.


