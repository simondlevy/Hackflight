# Running Hackflight on Teensy 3.2

As described in this [blog post](http://diydrones.com/profiles/blogs/hackflight-teensycopter), you can run 
Hackflight on the Teensy 3.2 Arduino-compatible microcontroller, but this will require additional hardware
purchases and wiring.

To build Hackflight for Teensy, copy everything in the <b>firmware</b> directory (above this one) to the 
<b>hackflight</b> directory (below this one), and rename <b>hackflight.cpp</b> to <b>hackflight.ino</b>.
You should then be able to launch the Arduino IDE and build and flash the firmware onto your Teensy.

