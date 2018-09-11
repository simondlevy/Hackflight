This folder contains experimental code to support running Hackflight on the popular STM32F-based flight controllers,
by linking to the low-level device drivers (UART, I<sup>2</sup>C, SPI, PWM) in
the Cleanflight repository. It is easiest to compile this code on Linux, using
the [GNU Arm Embedded
Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads).
After installing the toolchain, download
[Cleanflight](https://github.com/cleanflight/cleanflight).  Put the Cleanflight
download on your desktop, or edit the line in the
[Makefile](https://github.com/simondlevy/Hackflight/blob/master/extras/stm32f/alienflightf3v1/Makefile#L26-L28)
to reflect where you put it. Then cd to the folder for your board and type
<tt>make</tt>.

The following flight controllers are currently supported and have been tested:

* [Alienflight F3](http://www.readytoflyquads.com/alien-f3-brushed-flight-controller) brushed flight controller
controller from ReadyToFlyQuads

* [BetaFPV F3 brushed flight controller](https://betafpv.com/products/betafpv-f3-evo-whoop-brushed-flight-controller-no-receiver-version)

What these boards have in common is the ability to short the boot-loader pads easily, either through a jumper
or boot button.  



