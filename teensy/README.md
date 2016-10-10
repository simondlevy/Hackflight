# Running Hackflight on Teensy 3.2

As described in this [blog post](http://diydrones.com/profiles/blogs/hackflight-teensycopter), you can run 
Hackflight on the Teensy 3.2 Arduino-compatible microcontroller, but this will require additional hardware
purchases and wiring.

To build Hackflight for Teensy, you should first have the 
[Arduino software](https://www.arduino.cc/en/Main/Software) installed on your computer.  Since the
latest fully-supported version for Teensy is Arduino 1.6.11, don't go any higher than that.  Then install 
[Teensyduino](http://www.pjrc.com/teensy/td_download.html). Once you've done that, find the
<b>hardware/teensy/avr/libraries</b> folder under your Arduino folder, and install the following two libraries
there:
<ul>
<li> Bolderflight's MPU9250 [library](https://github.com/bolderflight/MPU9250)
<p><li> My ArduinoRXInterrupt [library](https://github.com/simondlevy/ArduinoRXInterrupt)
</ul>

Finally, copy everything in the <b>firmware</b> directory (above this one) to the 
<b>hackflight</b> directory (below this one), and rename <b>hackflight.cpp</b> to <b>hackflight.ino</b>.
You should then be able to launch the Arduino IDE and build and flash the firmware onto your Teensy.

