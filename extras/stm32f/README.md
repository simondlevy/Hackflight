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

