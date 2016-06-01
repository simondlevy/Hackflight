This directory contains code that you can use to build HackflightSim, a flight simulator that uses the
Hackflight firmware and the [Virtual Robot Experimentation Platform](http://www.coppeliarobotics.com/) 
from Coppelia Robotics.  

To build and run the simulator you will need download V-REP, use the Linux operating system, and have a Taranis 
transmitter with USB adapter cable.  If there is enough community interest, I will add support for other OSs and
controllers.

The simulator uses a V-REP plugin for optimal speed and simplicity.  To build the plugin you should clone the 
hackflight repository, then do

<pre>
cd hackflight/sim
make
</pre>

You can then either (1) drag the plugin library file <b>libv_repExtHackflight.so</b> to your V-REP folder, 
or (2) change the value of <tt>VREP\_DIR</tt> in the Makefile and then do

<pre>
make install
</pre>

You can then launch V-REP, open the scene file <b>hackflight.ttt</b>, and click the play button to start the
simuation.
