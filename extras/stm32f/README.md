This folder contains code to support running Hackflight on the popular STM32F-based flight controllers,
by linking to the low-level device drivers (UART, I<sup>2</sup>C, SPI, PWM) in
the Cleanflight repository. It is easiest to compile this code on Linux, doing the following:

1. Download [Cleanflight](https://github.com/cleanflight/cleanflight).  

2. Install curl (Ubuntu: <tt>sudo apt install curl</tt> Fedora: <tt>sudo yum install curl</tt>)

3. cd to Cleanflight directory and do <tt>make arm\_sdk\_install</tt>

4. In your <b>~/.bashrc</b> file, add the line <tt>export PATH=$PATH:$HOME/Desktop/cleanflight/tools/gcc-arm-none-eabi-7-2017-q4-major/bin</tt>  (assuming you put Cleanflight in your Desktop directory).

5. Edit the line in your [Makefile](https://github.com/simondlevy/Hackflight/blob/master/extras/stm32f/examples/alienflightf3v1_dsmx/Makefile#L26-L28)
to reflect where you put Cleanflight. Then cd to the folder for your board and type
<tt>make</tt>.

The following flight controllers are currently supported and have been tested:

* [Alienflight F3](http://www.readytoflyquads.com/alien-f3-brushed-flight-controller) brushed flight controller
controller

* [BetaFPV F3](https://betafpv.com/products/betafpv-f3-evo-whoop-brushed-flight-controller-no-receiver-version) brushed flight controller

* [Femto F3](http://www.readytoflyquads.com/f3-femto-flight-controller) brushless flight controller

* [Omnibus F3](https://www.readytoflyquads.com/flip-32-f3-omnibus) brushless flight controller

What these boards have in common is the ability to short the boot-loader pads easily, either through a jumper
or boot button.  



