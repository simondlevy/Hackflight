# Running Hackflight on the Ladybug Flight Controller

The code in the the directories below allows you to run Hackflight on the brushed-motor
[Ladybug Flight Controller](https://www.tindie.com/products/TleraCorp/ladybug-flight-controller/).
Each example works with a different receiver protocol (DSMX, SBUS, CPPM).
All three examples use the PID parameters that we've tuned for the popular 
[3DFly](https://www.thingiverse.com/thing:2519301) printable frame, which
should also work well for similarly-sized vehicles.  Our
[wiki](https://github.com/simondlevy/Hackflight/wiki) 
will take you through the steps of setting up this board and using it on a quadcopter.

The <b>motortest</b> directory contains a little Arduino sketch you can run to test the motors.
