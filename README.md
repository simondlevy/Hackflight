<p align="center"> 
<img src="extras/media/logo.png" width=400>
<br><i>Hackflight logo by MC Greenleaf</i>
</p>

Hackflight is simple, platform-independent, header-only C++ firmware for multirotor
[flight controllers](https://www.tindie.com/products/TleraCorp/ladybug-flight-controller/) 
and [simulators](https://github.com/simondlevy/HackflightSim).  It
is geared toward people like me who want to tinker with flight-control
firmware, and use it to teach students about ideas like inertial measurement
and PID tuning.  <b>If you are in the 99% percent of users who just want to get
your vehicle flying without getting into firmware hacking, I recommend
[Cleanflight](http://cleanflight.com/)</b> (great for getting started when
you're on a budget) <b>or the
[Ardupilot](http://copter.ardupilot.org) system</b> (for
sophisticated mission planning with waypoint navigation and the like).  In
addition to big user communities and loads of great features, these platforms
have safety mechanisms that Hackflight lacks, which will help avoid injury to
you and damage to your vehicle.

Although Hackflight is hardware/platform-independent, 
the hardware focus of the project has shifted to the Arduino-compatible, STM32L4-based 
boards from [Tlera Corp](https://www.tindie.com/stores/TleraCorp/): the
[Ladybug FC](https://www.tindie.com/products/TleraCorp/ladybug-flight-controller/)
brushed-motor board and the (currently in development) brushless-motor 
[ButterflyFC](https://github.com/simondlevy/Hackflight/wiki/B.01-Bill-of-Materials).
In addition to offering the full complement of i/o signals (UART, I<sup>2</sup>C,
SPI, digital, analog), these boards have hardware support for floating-point
operations, allowing us to write simpler code based on standard units:

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

Because a multirotor build typically involves choosing a flight-control board,
radio receiver, model (airframe), and PID control settings, Hackflight provides
a separate C++ class to support each of these components:
<ul>
<li> The <a href="https://github.com/simondlevy/Hackflight/blob/master/src/board.hpp">Board</a>
class specifies a set of four abstract (pure virtual) methods that you must
implement for a particular flight controller or simulator: getting the current
quaternion from the IMU; getting gyrometer rates from the IMU; sending
commands to the motors; and getting the current time.  <li> The <a
href="https://github.com/simondlevy/Hackflight/blob/master/src/receiver.hpp">Receiver</a>
class performs basic functions associated with R/C control (tracking stick
positions, checking switches) and specifies a set of abstract methods that you
implement for a particular receiver (reading channels values).  <li>The <a
href="https://github.com/simondlevy/Hackflight/blob/master/src/mixer.hpp">Mixer</a>
class is an abstract class that can be subclassed for various motor
configurations (QuadX, Hexacopter, Tricopter, etc.).  The <a
href="https://github.com/simondlevy/Hackflight/blob/master/src/mixers/quadx.hpp">QuadX</a>
subclass is already implemented.  
<li>The <a href="https://github.com/simondlevy/Hackflight/blob/master/src/stabilizer.hpp">Stabilizer</a>
class provides a constructor where you specify the stabilization PID values
appropriate for your model.  
</ul>

Because it is useful to get some visual feedback on things like vehicle orientation and RC receiver
channel values,  we also provide a very simple &ldquo;Ground Control Station&rdquo; (GCS) program.
that allows you to connect to the board and see what's going on. Windows users
can run this program directly: just download [this zipfile](http://home.wlu.edu/~levys/hackflight-gcs.zip),
unzip the file, open the folder, and double-click on <b>hackflight.exe</b>.
Others can run the <b>hackflight.py</b> Python script in the
<b>extras/gcs/python</b> folder.  To run the Python script you'll
need to install [MSPPG](https://github.com/simondlevy/hackflight/tree/master/extras/parser), a
parser generator for the Multiwii Serial Protocol (MSP) messages used by the
firmware. Follow the directions in that repository to install MSPPG for Python.

To support working with new new sensors and PID control algorithms, the <b>Hackflight</b> C++ class provides
two methods: <tt>addSensor</tt> and <tt>addPidController</tt>.   For an example of how to use these
methods, take a look at 
this
[sketch](https://github.com/simondlevy/Hackflight/blob/master/examples/Ladybug/LadybugDSMX_VL53L1X/LadybugDSMX_VL53L1X.ino),
which uses the [VL53L1X long-range proximity sensor](https://www.tindie.com/products/onehorse/vl53l1-long-range-proximity-sensor/)
to provide altitude hold.

To get started with Hackflight, take a look at the [build wiki](https://github.com/simondlevy/Hackflight/wiki).
To understand the principles behind the software, contniue reading.

## Design Principles

There are two basic data types in Hackflight: 
[state](https://github.com/simondlevy/Hackflight/blob/master/src/datatypes.hpp#L34-L44) and 
[demands](https://github.com/simondlevy/Hackflight/blob/master/src/datatypes.hpp#L25-L32).  For anyone
who's studied [Kalman filtering](http://home.wlu.edu/~levys/kalman_tutorial/), the state will be familiar:
it is the set of values that define the state of the vehicle at a given time
(altitude, orientation, angular velocity, ...), which gets modified by a set of
sensors (gyrometer, accelerometer, barometer, rangefinder, ...).  Once the
state has been determined, it is used by a set of [PID
controllers](https://en.wikipedia.org/wiki/PID_controller) to modify the
demands (throttle, roll, pitch, yaw) received by the R/C receiver or other
control device. Then the demands are
sent to the [mixer](https://github.com/simondlevy/Hackflight/blob/master/src/mixer.hpp), which determines the
values to be sent to each motor:

<img src="extras/media/dataflow.png" width=800>

As discussed above, Hackflight requires a bare minimum of 
[two sensor readings](https://github.com/simondlevy/Hackflight/blob/master/src/board.hpp#L40-L41)
&ndash; quaternion and gyrometer &ndash; and one PID controller, the 
[Stabilizer](https://github.com/simondlevy/Hackflight/blob/master/src/pidcontrollers/stabilizer.hpp).
Technically, the quaternion is more properly part of the vehicle state, but
because of the availability of &ldquo;hardware quaternion&rdquo; data from
modern sensors like the [EM7180 SENtral Sensor Fusion
Solution](https://www.tindie.com/products/onehorse/ultimate-sensor-fusion-solution-mpu9250/),
we find it convenient to treat the quaternion as a sensor reading.  For inertial measurement units
(IMUs) like the MPU9250 that do not deliver a hardware quaternion, Hackflight provides a 
pair of classes &ndash; 
[MadgwickQuaternionFilter](https://github.com/simondlevy/Hackflight/blob/master/src/filters.hpp#L94-L95)
and 
[MahonyQuaternionFilter](https://github.com/simondlevy/Hackflight/blob/master/src/filters.hpp#L201-L202)
 &ndash; that can be used to compute the quaternion on your microcontroller.

To provide access to other popular surface-mount sensors that you may wish to read, Hackflight also has classes to support
[accelerometers](https://github.com/simondlevy/Hackflight/blob/master/src/sensors/accelerometer.hpp), 
[magnetometers](https://github.com/simondlevy/Hackflight/blob/master/src/sensors/magnetometer.hpp), and 
[barometers](https://github.com/simondlevy/Hackflight/blob/master/src/sensors/barometer.hpp).  Together with the
[quaternion](https://github.com/simondlevy/Hackflight/blob/master/src/sensors/quaternion.hpp) and 
[gyrometer](https://github.com/simondlevy/Hackflight/blob/master/src/sensors/gyrometer.hpp), these are all sub-classes 
of the [SurfaceMountSensor](https://github.com/simondlevy/Hackflight/blob/master/src/sensors/surfacemount.hpp)
class, which is in turn a sub-class of the 
[Sensor](https://github.com/simondlevy/Hackflight/blob/master/src/sensor.hpp#L27-L37)
class.  The Sensor class is an abstract (virtual) class (a.k.a. interface) specifying two methods that any sensor
must implement: (1) reporting whether the sensor is ready to deliver new data;
(2) modifying the vehicle state.  By requiring each sensor to report its
readiness, we can avoid the need to write a separate timing loop for each
sensor in the main [loop code](https://github.com/simondlevy/Hackflight/blob/master/src/hackflight.hpp#L301-L312). To
implement additional sensors, you can directly sub-class the Sensor class, as we've done with the 
[Rangefinder](https://github.com/simondlevy/Hackflight/blob/master/src/sensors/rangefinder.hpp) 
class that we use to support the
[VL53L1](https://www.tindie.com/products/onehorse/vl53l1-long-range-proximity-sensor/) time-of-flight rangefinder 
in an [example sketch](https://github.com/simondlevy/Hackflight/blob/master/examples/Ladybug/LadybugDSMX_VL53L1X/LadybugDSMX_VL53L1X.ino#L60-L84).
