<img src="hackflight.png">

This directory contains code that you can use to build HackflightSim, a flight
simulator that uses the Hackflight firmware and the [Virtual Robot
Experimentation Platform](http://www.coppeliarobotics.com/) from Coppelia
Robotics.  

To build and run the simulator you will need download V-REP, and you will need to
use the Linux operating system.  (We are working on a version for Windows 10.)
The simulator works best with a PS3 controller, Taranis transmitter with USB
mini-B cable, or Spektrum transmitter with 
[GWS Adapter cable] (https://www.amazon.com/gp/product/B000RO7JAI)
(which is unfortunately no longer available).
You can also control it from the numeric keypad on your
keyboard, using the [key
mappings](http://www.flightsimbooks.com/flightsimhandbook/keyboardcontrols.php)
for the Microsoft Flight Simulator, but I've found it awkward to fly this way:
you have to focus the terminal window from which you launched V-REP in order
for the simulator to detect the keystrokes.  On the PS3, I have found it useful
to wiggle the sticks a bit when starting, to make sure the program is detecting
the controller.

The simulator uses a V-REP plugin for optimal speed and simplicity.  To build
the plugin you should clone the hackflight repository, cd to
<b>hackflight/sim</b>, and edit the value of <tt>VREP\_DIR</tt> in the Makefile
to reflect where you installed V-REP, as well as the value of
<tt>CONTROLLER</tt> appropriate for the controller you're using.  Typing
<tt>make install</tt> will build the plugin library file
<b>libv_repExtHackflight.so</b> and copy it to the V-REP folder. You should not
have V-REP running while you do this, or you'll likely cause it to shut itself
down.

You can then launch V-REP, open the scene file <b>hackflight.ttt</b>, and click
the play button to start the simuation.
