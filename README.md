# Hackflight: Simple quadcopter flight control firmware and simulation for C++ hackers

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
Multiwii).  Although there is [legacy support](https://github.com/simondlevy/hackflight/tree/master/legacy) 
for STM32F103 flight-controller boards
(Naze32 and clones like Flip32, MultiRC, etc.) and the Alienflight F3 board,
the hardware focus of the project has shifted to the Arduino-compatible boards designed
by Pesky Products: the [Teensy Flight Controller](https://forum.pjrc.com/threads/32985-Teensy-Flight-Controller)
and the STM32L4-based [Ladybug FC](http://diydrones.com/profiles/blogs/flight-of-the-ladybug).

Thanks to a major effort by
[Sytelus](https://github.com/sytelus), the core Hackflight 
[firmware ](https://github.com/simondlevy/hackflight/tree/master/include) now
adheres to best practices for C++.  As you can 
[see](https://github.com/simondlevy/hackflight/blob/master/boards/alienflightf3/hackflight.cpp), 
the code follows the Arduino
design pattern of a <tt>startup</tt> routine that calls the
<tt>init()</tt> method of a few objects (<tt>IMU</tt>, <tt>RC</tt>,
<tt>PID</tt>, <tt>Board</tt>) and a <tt>loop</tt> routine that calls the
<tt>update()</tt> method and other methods of those objects.  The code provides
abstraction (through the <tt>Board</tt> class) that should make it easy to use
on other boards.  The <tt>Board</tt> class declares the pure virtual methods that you must override
for implementation on a particular board or simulator, as well as a few &ldquo;extras&rdquo,
virtual methods that you can override for additional functionality like altitude-hold, hover-in-place,
etc.  Support for these extra methods can be found in the <tt>include/extras</tt> folder; for example,
there is a <tt>Baro</tt> class that performs typical functions of a barometer.

The only parameters you should need to adjust are the PID tuning 
[params](https://github.com/simondlevy/hackflight/blob/master/include/config.hpp#L25-L43). 
As usual, collective lower-right arms the board, and lower-left disarms it, as
indicated by the red LED.  The green LED will flash when the board is tilted by
more than 25 degrees.

Although Hackflight was designed to be &ldquo;headless&rdquo; (no configurator program),
it is useful to get some visual feedback on things like vehicle orientation and RC receiver
PWM values.  So in the <tt>gcs</tt> folder you'll find a Python program (<tt>hackflight.py</tt>)
that allows you to connect to the board and see what's going on.  To use this program you'll
need to install [MSPPG](https://github.com/simondlevy/hackflight/tree/master/parser), a
parser generator for the Multiwii Serial Protocol (MSP) messages used by the
firmware. Follow the directions in that repository to install MSPPG for Python.

If you find Hackflight useful, please consider donating
to the [Baseflight](https://goo.gl/3tyFhz) or 
[Cleanflight](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=TSQKVT6UYKGL6)
projects from which it is derived.
