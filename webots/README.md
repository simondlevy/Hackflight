<p align="center"> 
<img src="../media/webots2.png" width=450>
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
make run
```

If you have a game controller or R/C transmitter with adapter dongle, you can
use that to fly;  otherwise, the simulator will advise you that no such device
was found and instruct you on how to fly with the keyboard.  The following devices
are currently supported:

* [MGEAR Wired Controller for PS3](https://www.officedepot.com/a/products/7123231/Gear-Wired-Controller-For-PS3-Black/)
* [Logitech Gamepad F310](https://www.amazon.com/gp/product/B003VAHYQY)
* [Logitech Extreme 3D Pro](https://www.amazon.com/gp/product/B00009OY9U)
* [FrSky XSR-SIM USB Dongle](https://www.amazon.com/gp/product/B07GD6ZLW7)
* [Spektrum Ws2000 Wireless USB RC Flight Simulator Dongle](https://www.amazon.com/gp/product/B07ZK1R32H)
* Nyko 83069 Playstation(R)3 Core Wired Controller
