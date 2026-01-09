<p align="center"> 
<img src="media/logo.png" width=450>
</p>

# Intro

Hackflight is a minimalist software toolkit for building multirotor flight
controllers and simulators.  It is geared toward people like me who want to
tinker with flight-control firmware, and use it to teach students about ideas
like
[state estimation](https://simondlevy.github.io/ekf-tutorial)
and
[PID control](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller).
<b>If you are in the 99% percent of
users who just want to get your vehicle flying without getting into firmware
hacking, I recommend [Betaflight](http://betaflight.com/)</b> (great for
getting started when you're on a budget)
<b>the [Ardupilot](http://copter.ardupilot.org) system</b> (for
sophisticated mission planning with waypoint navigation and the like), or
the <b>[Crazyflie](https://www.bitcraze.io/products/old-products/crazyflie-2-1/)</b>
platform, for a safe, inexpensive introduction to quadcopters as a research
platform.   In addition to big user communities and loads of great features,
these platforms have safety mechanisms that Hackflight lacks, which will help
avoid injury to you and damage to your vehicle.

# Design principles

Hackflight attempts to maintain a simple relationship between
the code and the elements of the flight-control dataflow diagram shown below.
Boxes represent data, ovals represent functions, and feedback arrows
represent the need for functions that have state (instance variables); for
example, the maintenance of an error integral in a PID controller:

<img src="media/dataflow2.png" width=700>

By using header-only C++ classes whenever possible, avoiding C-style macros and 
[null pointers](https://www.infoq.com/presentations/Null-References-The-Billion-Dollar-Mistake-Tony-Hoare/),
and relying on existing Arduino libraries for sensors and actuators,
Hackflight supports a [composable](https://www.progress.com/blogs/what-composability-why-should-you-care) 
approach to taming the complexity of flight control. The Hackflight [codebase](src/) is
around 3,000 lines of code.

# Hardware

Hackflight currently runs on a custom-built micro [quadcopter](media/boltquad2.jpg) that uses 
the [Crazyfle Bolt 1.1](https://www.bitcraze.io/products/crazyflie-bolt-1-1/)
flight controller for state estimation and PID control, and a
[BlueSMIRF v2](https://www.sparkfun.com/sparkfun-bluesmirf-v2.html) for 
remote control and logging.

# Simulator

For flight simulation, Hackflight uses [Webots](https://cyberbotics.com/),
a free, open-source robotics simulator. Click [here](webots) to get started.

# Citing Hackflight

Please cite Hackflight as:

```
@ARTICLE{10.3389/fnbot.2020.00016,
AUTHOR={Levy, Simon D.},   
TITLE={Robustness Through Simplicity: A Minimalist Gateway to Neurorobotic Flight},      
JOURNAL={Frontiers in Neurorobotics},      
VOLUME={14},           
YEAR={2020},      
URL={https://www.frontiersin.org/articles/10.3389/fnbot.2020.00016},       
DOI={10.3389/fnbot.2020.00016},      
ISSN={1662-5218}
}
```
