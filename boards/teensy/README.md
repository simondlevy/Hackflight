# Running Hackflight on Teensy 3.2

The code in the hackflight directory below allows you to run Hackflight on Kris Winer's brushed-motor 
[flight controller](https://forum.pjrc.com/threads/32985-Teensy-Flight-Controller)
based on Teensy 3.1.

To build Hackflight for Teensy, you should first have the 
[Arduino software](https://www.arduino.cc/en/Main/Software) installed on your computer.  (Make sure to install
a version no higher than the latest one supporting Teensy.) Then install 
[Teensyduino](http://www.pjrc.com/teensy/td_download.html). Once you've done that, find the
<b>hardware/teensy/avr/libraries</b> folder under your Arduino folder, and install the following two libraries
there:
<ul>
<p><li> My [EM7180_passthru](https://github.com/simondlevy/EM7180_passthru) library
<p><li> My [SpektrumDSM](https://github.com/simondlevy/SpektrumDSM) library
</ul>

Finally, copy everything in the <b>include</b> directory (two levels above this one) to the 
<b>hackflight</b> directory (below this one).  You should then be able to
launch the Arduino IDE and build and flash the firmware onto your Teensy.

