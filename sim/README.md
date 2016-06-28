# HackflightSim: A cross-platform flight simulator using the Hackflight firmware

<img src="hackflight.png">

<b>Quickstart, Windows</b>

<ol>
<li> Install <a href="http://www.coppeliarobotics.com/downloads.html">V-REP</a>.
<p><li>Clone the hackflight repository.
<p><li>Drag the plugin <b>hackflight/sim/Release/v_repExtHackflight.dll</b>
to the V-REP folder where the other DLLs are. On my computer this is
<b>C:\Program FIles (x86)\V-REP3\V-REP_PRO_EDU</b>.
<p><li>Plug in your R/C transmitter, joystick, or game controller. Currently supported:
<p><ul>
<li> FrkSky Taranis transmitter with USB cable
<p><li>Spektrum DX-8 transmitter with GWS adapter cable (unfortunately no longer sold)
<p><li>Logitch Extreme 3D Pro joystick
<p><li>Nyko Core Controller for PS3
<p><li>Keyboard, using the Microsoft Flight Controller numeric keypad 
<a href="http://www.flightsimbooks.com/flightsimhandbook/keyboardcontrols.php">mappings</a>,
if none of the above are connected: not recommended because it requires you 
to foreground the console
</ul>

<p><li> Double-click on the hackflight scene <b>hackflight.ttt</b> to launch V-REP.
<p><li> Press the triangular play button at the top of V-REP.
<p><ul>
<p><li> If V-REP asks you whether you really want to run the simulation with a dt=10msec
update rate, agree.  You may have to do this three times.
<p><li>If the vehicle takes off immediately, make sure your throttle is down by
raising it slightly and then lowering it all the way down.  You may also need to wiggle
the sticks a little at first for the simulator to detect them.
</ul>
<p><li>Press the square stop button at the top of V-REP to stop the simulation.
</ol>


<b>Quickstart, Linux</b>

<ol>
<li> Install <a href="http://www.coppeliarobotics.com/downloads.html">V-REP</a>.
<p><li>Clone the hackflight repository.
<p><li>Drag the plugin <b>hackflight/sim/Release/libv_repExtHackflight.so</b>
to the V-REP folder where the other plugins are. On my computer this is
<b>~/Software//V-REP_PRO_EDU_V3_3_1_64_Linux</b>.
<p><li>Plug in your R/C transmitter, joystick, or game controller (see above).
<p><li> Open a terminal window, navigate to the V-REP folder, and do <b>./vrep.sh</b> to launch V-REP.
<p><li> Follow steps 6-7 above.
</ol>

<b>Quickstart, Mac OS X</b>
<b><i>Note: The simulator runs much slower on OS X for some reason.</i></b>
<ol>
<li> Install <a href="http://www.coppeliarobotics.com/downloads.html">V-REP</a>.
<p><li> Install the <a href="https://www.libsdl.org/release/SDL2-2.0.4.dmg">SDL runtime binaries</a> 
in your <b>/Library/Frameworks</b> folder.
<p><li>Clone the hackflight repository.
<p><li>Drag the plugin <b>hackflight/sim/Release/libv_repExtHackflight.dylib</b>
to the V-REP folder where the other plugins are. On my computer this is
<b>/Applications/V-REP_PRO_EDU_V3_3_1_Mac/vrep.app/Contents/MacOS</b>
<p><li>Plug in your R/C transmitter, joystick, or game controller (see above).
<p><li> Double-click on the vrep icon in the location where you installed V-REP.  
On my computer this is <b>/Applications/V-REP_PRO_EDU_V3_3_1_Mac</b>
<p><li> Follow steps 6-7 above.
</ol>


<b>Developing for Windows</b>

You will need Visual Studio for C++.  (I installed the free Visual C++ 2010 Express.) After cloning 
the hackflight repository, double-click on
<b>v_repExtHackflight.vcxproj</b> to launch the project.  Building for Release will create the
DLL file, which you can then move into the V-REP folder as in the Windows quickstart above.

<b>Developing for Linux and Mac OS X</b>

After cloning the hackflight repository, edit the <tt>VREP\_DIR</tt> variable in
the Makefile in this directory (<b>hackflight/sim</b>) to reflect where you
installed V-REP.  Typing <b>make install</b> should then build the plugin and
install it the appropriate place.

