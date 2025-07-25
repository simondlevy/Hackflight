Additional Arduino libraries needed:

* https://github.com/simondlevy/Arduino_CMSIS-DSP
* https://github.com/simondlevy/BoschSensors
* https://github.com/simondlevy/PMW3901
* https://github.com/simondlevy/VL53L1

In ~/.arduino15/packages, put STMicroelectronics, with tools/CMSIS/5.9.0/CMSIS/ removed

Then:

```make standard_config```

or


```make haskell_config```

then

```make -j 32```

