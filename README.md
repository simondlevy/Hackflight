<p align="center"> 
<img src="media/logo.png" width=450>
</p>

## Intro

Hackflight is a software toolkit for building multirotor flight
controllers and simulators.  It is geared toward people like me who want to tinker with
flight-control firmware, and use it to teach students about ideas like inertial
measurement and PID tuning.  <b>If you are in the 99% percent of users who just
want to get your vehicle flying without getting into firmware hacking, I
recommend [Betaflight](http://betaflight.com/)</b> (great for getting started
when you're on a budget, and the origin of much of the code in Hackflight)
<b>or the [Ardupilot](http://copter.ardupilot.org) system</b> (for
sophisticated mission planning with waypoint navigation and the like).  In
addition to big user communities and loads of great features, these platforms
have safety mechanisms that Hackflight lacks, which will help avoid injury to
you and damage to your vehicle.


## Supported platforms

To make it easy to develop flight firmware using Arduino-compatible
microcontrollers, Hackflight is laid out as an Arduino library; however,
this does not limit your ability to use it on other hardware platforms 
and simulators.

Currently, Hackflight has been tested on the following platforms:

* [Ladybug Flight Controller](https://www.tindie.com/products/TleraCorp/ladybug-flight-controller) from Tlera Corp.

* [STM32F405 AIO flight controller](https://betafpv.com/products/f405-20a-aio-2-4s-brushless-flight-controllerblheli_s-v3)

* [MulticopterSim](https://github.com/simondlevy/MulticopterSim) flight simulator based on UnrealEngine

Simulators can use just the <b>.h</b> files in the
[src](https://github.com/simondlevy/Hackflight/tree/master/src) directory, which contain the
basic control algorithms (open-loop control / PID control).  Real flight
controllers use the code in the <b>.c</b> files located in that directory
directory and in the [stm32f4/src](https://github.com/simondlevy/Hackflight/tree/master/stm32f4/src) directory,
which contain code for real-time scheduling and for reading from physical sensors and R/C receivers.

## Design principles

Although previously implemented in C++, Hackflight has shifted toward a focus on functional-style
programming in C. For example, instead of having a Receiver class that gets subclassed for different
kinds of receivers (SBUS, DSMX), you pass the appropriate device-access
functions to the the code that retrieve the raw values from the receiver.  

More generally, Hackflight attempts to maintain a simple relationship between
the code and the elements of the flight-control flow diagram shown below:

<p align="center"> 
<img src="media/dataflow2.png" width=700>
</p>

## Ground Control Station

Because it is useful to get some visual feedback on things like vehicle orientation and RC receiver
channel values,  we also provide a very simple &ldquo;Ground Control Station&rdquo; (GCS) program
that allows you to connect to the board and see what's going on. Windows users
can run this program directly: just clone the [HackflightGCS](https://github.com/simondlevy/HackflightGCS)
repository and double-click on <b>hackflight.exe</b>.  Others can run the
<b>gcs.py</b> Python script in the <b>gcs</b> folder.  

## PID Controllers

By default, Hackflight has a single PID controller, for controlling the vehicle's roll,
pitch, and yaw angles via four constants: K<sub>P</sub>, K<sub>I</sub>, and K<sub>D</sub> for
angular velocity, and another K<sub>P</sub> for roll and
pitch angle values     If the latter value is set to zero,
the vehicle will fly in so-called &ldquo;rate mode&rdquo;; otherwise, it will fly in
&ldquo;level mode&rdquo;.

<p align="center"> <img src="media/pidcontrollers.png" width=600> </p>
