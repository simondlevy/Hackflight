<p align="center"> 
<img src="../media/webots.png" width=450>
</p>

# Hackflight simulator instructions

1. [Install Webots](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)
on your computer.  

2. Add the following line to your ```~/.bashrc``` file:

```
  export WEBOTS_HOME=/usr/local/webots
```

3. From the hackflight main directory, do the following:

```
cd webots/controllers/cplusplus
make
cd ../..
make cplusplus
```

If you have a game controller or R/C transmitter with adapter dongle, you can
use that to fly;  otherwise, the simulator will advise you that no such device
was found and instruct you on how to fly with the keyboard.  The following devices
are currently supported:

* MY-POWER CO.,LTD. 2In1 USB Joystick
* SHANWAN Android Gamepad
* Logitech Gamepad F310
* Logitech Extreme 3D
* FrSky Simulator
* Horizon Hobby SPEKTRUM RECEIVER
 
If you prefer to program in Python, there is also support for that; simply type ```make python```
instead of ```make cplusplus```.   You can now fly the vehicle with your keyboard (I don't currrently
support game controllers and R/C transmitters in Python).  The
```webots/controllers/python``` directory has code you can look at and modify.



