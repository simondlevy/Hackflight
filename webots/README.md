<p align="center"> 
<img src="../media/webots.png" width=450>
</p>

# Hackflight simulator instructions

1. [install Webots](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)
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
was found and instruct you on how to fly with the keyboard.
```



