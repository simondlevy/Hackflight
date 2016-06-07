# Hackflight: Simple quadcopter flight control firmware and simulator for C++ hackers

Hackflight is simple C++ firmware for inexpensive quadcopter flight
controllers.  It is geared toward people like me who want to tinker with
flight-control firmware, and use it to teach students about ideas like inertial
measurement and PID tuning.  <b>If you are in the 99% percent of users who just
want to get your vehicle flying without getting into firmware hacking, I
recommend [Cleanflight](http://cleanflight.com/)</b> (great for getting started
when you're on a budget) <b>or the
[Ardupilot](http://copter.ardupilot.org/ardupilot/index.html) system</b> (for
sophisticated mission planning with waypoint navigation and the like).  In
addition to big user communities and loads of great features, these platforms
have safety mechanisms that Hackflight lacks, which will help avoid injury to
you and damage to your vehicle.

Hackflight derives from the Baseflight firmware (which in turn derives from
Multiwii), and currently works only on STM32F103 flight-controller boards
(Naze32 and clones like Flip32, MultiRC, etc.) The original Hackflight idea was to
write firmware for flight controllers built from the Arduino / Teensy hardware.
As you'll see, the code follows the Arduino design pattern of a <tt>startup</tt>
routine that calls the <tt>init()</tt> method of a few objects (<tt>IMU</tt>,
<tt>RC</tt>, <tt>PID</tt>, <tt>Board</tt>) and a <tt>loop</tt> routine that
calls the <tt>update()</tt> method and other methods of those objects.
But with all the features you can now get onboard an inexpensive STM32F103
board (barometer, magnetometer, flash RAM), there seems little point in building
your own board, unless you're interested in hardware hacking.  So Hackflight currently
supports only the STM32F103 boards, while keeping the Arduino design principles.  The 
code provides abstraction (through the <tt>Board</tt> class) that should 
make it easy to use on other boards.

Meanwhile, to try Hackflight on your board, you'll need to be running Linux on your
desktop/laptop computer, with the [GNU ARM
toolchain](https://launchpad.net/gcc-arm-embedded) installed, and you'll need
to grab the [BreezySTM32](https://github.com/simondlevy/BreezySTM32)
repository.  

In <tt>hackflight/firmware/naze130mm</tt> there's code that uses PID values (<tt>pidvals.hpp</tt>)
appropriate that worked well on my 130mm quadcopter.  Likewise, <tt>hackflight/firmware/naze250mm</tt> 
contains code that uses values that worked on a 250mm quad.   So choose whichever is closest to your 
vehicle, cd to that folder, and edit the Makeke to reflect where you put BreezySTM32. Then type <tt>make</tt>,
which will build the firmware binary in the <tt>obj</tt> directory.  If you've
already got Baseflight or Cleanflight running on your board, you should then
just be able to type <tt>make flash</tt> to flash Hackflight onto it.  If you
run into trouble, you can short the bootloader pins and type <tt>make unbrick</tt>.

Hackflight flies your quadcopter in Level (a.k.a. Stable) mode.  So the only parameters you
should need to adjust are the PID tuning params.
As with Baseflight, you get a gyro auto-calibration sequence on startup, indicated by  steady
green LED that turns off when the calibration is done.  You can re-calibrate
the gyro by putting the collective (left) stick in full upper-left and the
cyclic (right) in full center-down position.  You can calibrate the
accelerometer with collective lower-left and cyclic center-down.  As usual, collective 
lower-right arms the board, and lower-left disarms it, as indicated by the red LED.
The green LED will flash when the board is tilted by more than 25 degrees.

Although Hackflight was designed to be &ldquo;headless&rdquo; (no configurator program),
it is useful to get some visual feedback on things like vehicle orientation and RC receiver
PWM values.  So in the <tt>gcs</tt> folder you'll find a Python program (<tt>main.py</tt>)
that allows you to connect to the board and see what's going on.  To use this program you'll
need to install [MSPPG](https://github.com/simondlevy/MSPPG), a parser generator for the Multiwii Serial 
Protocol (MSP) messages used by the firmware. Follow the directions in that repository to install
MSPPG for Python.

If you find Hackflight useful, please consider donating
to the [Baseflight](https://goo.gl/3tyFhz) or 
[Cleanflight](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=TSQKVT6UYKGL6)
projects from which it is derived.


