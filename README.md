<p align="center"> 
<img src="extras/media/logo.png" width=450>
</p>

Hackflight is a simple, platform-independent, header-only C++ toolkit for
building multirotor flight controllers.  It is geared toward people like
me who want to tinker with flight-control firmware, and use it to teach
students about ideas like inertial measurement and PID tuning.  <b>If you are
in the 99% percent of users who just want to get your vehicle flying without
getting into firmware hacking, I recommend
[Betaflight](http://betaflight.com/)</b> (great for getting started when
you're on a budget) <b>or the [Ardupilot](http://copter.ardupilot.org)
system</b> (for sophisticated mission planning with waypoint navigation and the
like).  In addition to big user communities and loads of great features, these
platforms have safety mechanisms that Hackflight lacks, which will help avoid
injury to you and damage to your vehicle.

Hackflight is currently working on the following platforms:

* [TinyPICO](https://www.tinypico.com)

* [Ladybug](https://www.tindie.com/products/TleraCorp/ladybug-flight-controller/) brushed flight controller
from Tlera Corp.

* [SuperFly](https://www.tindie.com/products/onehorse/superfly-hackable-esp8266-flight-controller/) 
Hackable ESP8266 Flight Controller from Pesky Products

* [Butterfly DIY](https://diydrones.com/profiles/blogs/hackhawk-ii-an-arduino-compatible-brushless-flight-controller)
brushless flight controller (components from from Tlera Corp. and Pesky Products)

* [MulticopterSim](https://github.com/simondlevy/MulticopterSim) flight simulator based on UnrealEngine4

By supporting floating-point operations, these platforms allow us to write simpler code based on standard units:

* Distances in meters
* Time in seconds
* Quaternions in the interval [-1,+1]
* Euler angles in radians
* Accelerometer values in Gs
* Barometric pressure in Pascals
* Stick demands in the interval [-1,+1]
* Motor demands in [0,1]

Thanks to some help from [Sytelus](https://github.com/sytelus), the core
Hackflight
[firmware](https://github.com/simondlevy/hackflight/tree/master/src)
adheres to standard practices for C++, notably, short, simple methods and
minimal use of compiler macros like <b>#ifdef</b> that can make it difficult to
follow what the code is doing.  

## RoboFirmwareToolkit

Hackflight is built on top of
[RoboFirmwareToolkit](https://github.com/simondlevy/RoboFirmwareToolkit) (RFT),
a general-purpose toolkit for building robot firmware.  So before trying out Hackflight
you should also install RFT.  It is also worth checking out the README for RFT in order
to see some of the design principles supporting Hackflight (see also the note on PID
controllers below.)

## Ground Control Station

Because it is useful to get some visual feedback on things like vehicle orientation and RC receiver
channel values,  we also provide a very simple &ldquo;Ground Control Station&rdquo; (GCS) program.
that allows you to connect to the board and see what's going on. Windows users
can run this program directly: just download
[this zipfile](https://simondlevy.academic.wlu.edu/files/software/hackflight-gcs.zip),
unzip the file, open the folder, and double-click on <b>hackflight.exe</b>.
Others can run the <b>hackflight.py</b> Python script in the
<b>extras/gcs/python</b> folder.  To run the Python script you'll
need to install [MSPPG](https://github.com/simondlevy/hackflight/tree/master/extras/parser), a
parser generator for the Multiwii Serial Protocol (MSP) messages used by the
firmware. Follow the directions in that folder to install MSPPG for Python.

## PID Controllers

Note these two important points about PID controllers in Hackflight:

1. <b>A PID controller is not the same as a
[flight mode](https://oscarliang.com/rate-acro-horizon-flight-mode-level/).</b>
For example, so-called [Acro mode](http://ardupilot.org/copter/docs/acro-mode.html#acro-mode) 
requires a PID controller based on angular
velocity (a.k.a. rate, computed from the gyrometer) for each of the three angles
(roll, pitch yaw). So-called [Stabilize](http://ardupilot.org/copter/docs/stabilize-mode.html#stabilize-mode) 
mode requires these three angular-velocity controllers,
plus a PID controller based on angle (computed from the quaternion) for the
roll and pitch axes.   To support this arrangement in Hackflight, PID
controllers for aux state 0 will also run in aux states 1 and 2, and PID
controllers for aux state 1 will also run in aux state 2.

2. <b>It matters in which order you add PID controllers</b>, because the output
of one PID controller is the input to the next.  For example, to get Stabilize
mode, you want the Level controller to go first, setting the desired pitch/roll
angles, and the Rate controller to go next, to control the rate at which the
desired angle will be reached.

<p align="center"> <img src="extras/media/pidcontrollers.png" width=600> </p>
