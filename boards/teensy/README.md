# Running Hackflight on Teensy 3.2

The code in board.cpp allows you to run Hackflight on Kris Winer's brushed-motor 
(flight controller)[https://forum.pjrc.com/threads/32985-Teensy-Flight-Controller]
based on Teensy 3.2.

To build Hackflight for Teensy, you should first have the 
[Arduino software](https://www.arduino.cc/en/Main/Software) installed on your computer.  (Make sure to install
a version no higher than the latest one supporting Teensy.) Then install 
[Teensyduino](http://www.pjrc.com/teensy/td_download.html). Once you've done that, find the
<b>hardware/teensy/avr/libraries</b> folder under your Arduino folder, and install the following two libraries
there:
<ul>
<li> Bolderflight's MPU9250 <a href="https://github.com/bolderflight/MPU9250">library</a>
<p><li> My ArduinoRXInterrupt <a href="https://github.com/simondlevy/ArduinoRXInterrupt">library</a>
</ul>

Finally, copy everything in the <b>firmware</b> directory (above this one) to the 
<b>hackflight</b> directory (below this one), and rename <b>hackflight.cpp</b> to <b>hackflight.ino</b>.
You should then be able to launch the Arduino IDE and build and flash the firmware onto your Teensy.

