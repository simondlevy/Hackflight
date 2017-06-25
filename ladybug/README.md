# Running Hackflight on the Ladybug Flight Controller

The code in the hackflight directory below allows you to run Hackflight on Kris Winer's brushed-motor 
[flight controller](https://www.oshpark.com/shared_projects/HwGC0Gj3)
based on his STM32L432 "Ladybug" design.

To build Hackflight for Ladybug, you should first have the 
[Arduino software](https://www.arduino.cc/en/Main/Software) installed on your computer.  (Make sure to install
a version no higher than the latest one supporting Ladybug.) Then install 
[arduino-STM32L4](https://github.com/GrumpyOldPizza/arduino-STM32L4). Once you've done that, find the
<b>hardware/teensy/avr/libraries</b> folder under your Arduino folder, and install the following two libraries
there:
<ul>
<p><li> My [EM7180](https://github.com/simondlevy/EM7180) library
<p><li> My [SpektrumDSM](https://github.com/simondlevy/SpektrumDSM) library
</ul>

Finally, copy everything in the <b>include</b> directory (two levels above this one) to the 
<b>hackflight</b> directory (below this one).  You should then be able to
launch the Arduino IDE and build and flash the firmware onto your Ladybug.
