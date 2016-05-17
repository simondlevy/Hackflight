# Hackflight: Simple quadcopter flight control firmware for C++ hackers

Hackflight is simple C++ firmware for inexpensive quadcopter flight controllers.  It is geared toward people like me 
who want to tinker with flight-control firmware, and use it to teach students about ideas like inertial measurement 
and PID tuning.  <b>If you are in the 99% percent of users who just want to get your vehicle flying without getting
into firmware hacking, I recommend [Cleanflight](http://cleanflight.com/) (great for getting started when you're on a 
budget) or the [Ardupilot](http://copter.ardupilot.org/ardupilot/index.html) system</b> (for sophisticated mission 
planning with waypoint navigation and the like).  These platforms have community-tested safety features that
Hackflight lacks, which will help avoid injury and damage to your vehicle.

Hackflight derives from the Baselfight firmware (which in turn derives from Multiwii), and currently works only on 
STM32F103 flight-controller boards
(Naze32 and clones like Flip32, MultiRC, etc.) I had originally planned to write firmware for flight
controllers built from the Arduino / Teensy hardware, But with all the features you can now get onboard an
inexpensive STM32F103 board (barometer, magnetometer, flash RAM), I can't see the point of building your
own board, unless you're interested in hardware hacking.

If you find Hackflight useful, please consider donating
to the [Baseflight](https://goo.gl/3tyFhz) or 
[Cleanflight](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=TSQKVT6UYKGL6)
projects from which it is derived.


