This directory contains code that you can use to build HackflightSim, a flight simulator that uses the
Hackflight firmware and the [Virtual Robot Experimentation Platform](http://www.coppeliarobotics.com/) 
from Coppelia Robotics.  

To build and run the simulator you will need download V-REP, you will need to
use the Linux operating system, and have a PS3 controller or Taranis
transmitter with USB adapter cable.  (I am working on adding keyboard support.)
On the PS3, I have found it useful to wiggle the sticks a bit when starting, to
make sure the program is detecting the controller.

The simulator uses a V-REP plugin for optimal speed and simplicity.  To build the plugin you should clone the 
hackflight repository, cd to <b>hackflight/sim</b>, and edit the value of <tt>VREP\_DIR</tt> in the Makefile
to reflect where you installed V-REP.  Typing <tt>make install</tt> will build the plugin library file 
<b>libv_repExtHackflight.so</b> and copy it to the V-REP folder.

You can then launch V-REP, open the scene file <b>hackflight.ttt</b>, and click the play button to start the
simuation.
