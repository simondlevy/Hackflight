# Running Hackflight on STM32 F103 flight controllers (Naze32, Flip32, Beef's Brushed Board)

To try Hackflight on one of these boards, you'll need to be running Linux on
your desktop/laptop computer, with the [GNU ARM
toolchain](https://launchpad.net/gcc-arm-embedded) installed, and you'll need
to grab the [BreezySTM32](https://github.com/simondlevy/BreezySTM32)
repository.  

In the <tt>naze/130mm</tt> directory there's code that uses PID values (<tt>pidvals.hpp</tt>)
that worked well on my 130mm brushless quadcopter.  Likewise,
<tt>hackflight/boards/naze/250mm</tt> contains code that uses values that
worked on a 250mm quad.  The <tt>beef</tt> directory has code that worked with a 3DFly 
110mm copter using Beef's Brushed Board and 8.5mm brushed motors.
So choose whichever is closest to your vehicle, cd to
that folder, and edit the Makeke to reflect where you put BreezySTM32. Then
type <tt>make</tt>, which will build the firmware binary in the <tt>obj</tt>
directory.  If you've already got Baseflight or Cleanflight running on your
board, you should then just be able to type <tt>make flash</tt> to flash
Hackflight onto it.  If you run into trouble, you can short the bootloader pins
and type <tt>make unbrick</tt>.
