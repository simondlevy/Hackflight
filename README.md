# Hackflight: Simple quadcopter flight control firmware for C++ hackers

Hackflight is simple C++ firmware for inexpensive quadcopter flight
controllers.  It is geared toward people like me who want to tinker with
flight-control firmware, and use it to teach students about ideas like inertial
measurement and PID tuning.  <b>If you are in the 99% percent of users who just
want to get your vehicle flying without getting into firmware hacking, I
recommend [Cleanflight](http://cleanflight.com/)</b> (great for getting started
when you're on a budget) <b>or the [Ardupilot](http://copter.ardupilot.org/ardupilot/index.html) system</b> (for
sophisticated mission planning with waypoint navigation and the like).  In addition to big user communities
and loads of great features, these platforms have safety mechanisms that Hackflight lacks, which
will help avoid injury to you and damage to your vehicle.

Hackflight derives from the Baselfight firmware (which in turn derives from
Multiwii), and currently works only on STM32F103 flight-controller boards
(Naze32 and clones like Flip32, MultiRC, etc.) I had originally planned to
write firmware for flight controllers built from the Arduino / Teensy hardware,
But with all the features you can now get onboard an inexpensive STM32F103
board (barometer, magnetometer, flash RAM), I can't see the point of building
your own board, unless you're interested in hardware hacking.  So Hackflight currently
supports only the STM32F103 boards.  It provides abstraction (through the
<tt>Board</tt> class) that should make it easy to use the code for other boards.

Meanwhile, to try Hackflight on your board, you'll need to be running Linux on your
desktop/laptop computer, with the [GNU ARM toolchain](https://launchpad.net/gcc-arm-embedded)
installed, and you'll need to grab the 
[BreezySTM32](https://github.com/simondlevy/BreezySTM32) repository.  Then edit the
Makefile in <b>hackflight/firmware/naze</b> to reflect where you put BreezySTM32.
In a terminal window, cd to <b>hackflight/firmware/naze</b> and type <tt>make</tt>.
This will build the firmware in the <b>obj</b> directory.  If you've already got
Baselfight or Cleanflight running on your board, you should then just be able
to type <tt>make flash</tt> to flash Hackflight onto it.  If you run into trouble,
you can short the bootloader pins and type <tt>make unbrick</tt.


If you find Hackflight useful, please consider donating
to the [Baseflight](https://goo.gl/3tyFhz) or 
[Cleanflight](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=TSQKVT6UYKGL6)
projects from which it is derived.


