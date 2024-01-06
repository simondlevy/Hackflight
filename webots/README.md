# Running Hackflight in the Webots simulator

1. Install [Webots](https://cyberbotics.com/)

2. Launch Webots

3. From the File menu, chose <b>Open World</b>

4. Use the file dialog to open <b>Hackflight/webots/words/simple.wbt</b>

5. Click on the little [file-folder icon](media/screenshot.png) in the upper right,
and use the file dialog to open <b>controllers/simple/simple.cpp</b>

6. From the <b>Build</b> menu, choose <b>Build</b> to build the simulation.

Unless you've got a supported game controller plugged in, the output console at the 
bottom will show a warning that no free joystick was found.  You can simply
ignore this warning and fly the sim using the keyboard, as shown by the instructions
in the console. Currently supported game controllers are:

1. Spektrum R/C transmitters, via 
[Spektrum Wireless Flight Simulator Dongle](https://www.amazon.com/dp/B07ZK1R32H?ref=ppx_yo2ov_dt_b_product_details&th=1)


2. FrSky R/C/ transmitters, via 
[FrSky XSR-SIM Wireless USB Dongle](https://www.getfpv.com/frsky-xsr-sim-wireless-usb-dongle-for-simulators.html)

3. Various PS3 and XBox wired game controllers.  If you have one of these and you
get a message saying it wasn't recognized, create a new issue with that info, and
I'll try to get it working.

For gamepad controllers like PS3 and XBox with a springy throttle, you can use
the right shoulder button to enter hover mode.  For all other kinds of controllers
and the keyboard, you are always in hover mode.

