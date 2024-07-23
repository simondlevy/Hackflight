<p align="center"> 
<img src="media/logo.png" width=450>
</p>

## Intro

Hackflight is a minimalist software toolkit for building multirotor flight
controllers and simulators.  It is geared toward people like me who want to
tinker with flight-control firmware, and use it to teach students about ideas
like inertial measurement and PID tuning.  <b>If you are in the 99% percent of
users who just want to get your vehicle flying without getting into firmware
hacking, I recommend [Betaflight](http://betaflight.com/)</b> (great for
getting started when you're on a budget, and the origin of much of the code in
Hackflight) <b>or the [Ardupilot](http://copter.ardupilot.org) system</b> (for
sophisticated mission planning with waypoint navigation and the like).  In
addition to big user communities and loads of great features, these platforms
have safety mechanisms that Hackflight lacks, which will help avoid injury to
you and damage to your vehicle.

Hackflight is currently supported in Linux only.

## Design principles

Hackflight attempts to maintain a simple relationship between
the code and the elements of the flight-control dataflow diagram shown below.
Boxes represent data, ovals represent functions, and feedback arrows
represent the need for functions that have state (instance variables); for
example, the maintenance of an error integral in a PID controller:

<p align="center"> 
<img src="media/dataflow.png" width=700>
</p>

By using header-only C++ classes whenever possible, avoiding C-style macros and 
[null pointers](https://www.infoq.com/presentations/Null-References-The-Billion-Dollar-Mistake-Tony-Hoare/),
and leveraging existing Arduino libraries for 
[receivers](https://github.com/bolderflight/sbus),
[ESCs](https://github.com/simondlevy/DshotSTM32), and
[IMUs](https://github.com/simondlevy/MPU6x00),
Hackflight supports a [composable](https://www.programmingtalks.org/talk/brian-beckman-dont-fear-the-monad) 
approach to taming the complexity of flight control: you instantiate a Board
subclass, passing it your IMU settings, PID controllers, mixer, ESC type, and LED
pin number.  In your ```loop``` function, you just call the ```step()``` method
on the Board object, passing it the raw values from your IMU.  Look at this
[example
program](https://github.com/simondlevy/Hackflight/blob/master/examples/MambaF411Dsmx/MambaF411Dsmx.ino)
to get an idea of how this approach works.

## Simulator

For flight simulation, Hackflight uses [Webots](https://cyberbotics.com/),
a free, open-source robotics simulator. Click [https://github.com/simondlevy/hackflight/webots](here) to get started.

## Haskell support

[Why Haskell?](https://koerbitz.me/posts/Why-I-love-Haskell.html)

To experiment with using Hackflight for flight control in Haskell, you'll first
need to install [Haskell](https://www.haskell.org/) and [NASA
Copilot](https://copilot-language.github.io) (the &ldquo;secret sauce&rdquo;
that allows you to compile Haskell code to a form suitable for running on a
flight controller.)  I was able to do this via:

```
cabal install copilot
cabal install copilot-c99
```




## Citing Hackflight

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
