## Building the Hackflight firmware for the Crazyflie Bolt 1.1 flight controller

<img src="../media/boltquad2.jpg" width=800>

1. Install [Arduino](https://www.arduino.cc/en/software/) and the
   [Arduino Command-Line Interface (CLI)](https://docs.arduino.cc/arduino-cli/installation/)

2. Install [Arduino_Core_STM32](https://github.com/stm32duino/Arduino_Core_STM32/wiki/Getting-Started)

3. Install these Arduino libraries:

* https://github.com/stm32duino/STM32FreeRTOS
* https://github.com/simondlevy/Arduino_CMSIS-DSP (note installation instructions)
* https://github.com/simondlevy/PMW3901
* https://github.com/simondlevy/OneShot125
* https://github.com/simondlevy/PMW3901
* https://github.com/bolderflight/bmi088-arduino
* https://github.com/pololu/vl53l1x-arduino

4. In hackflight/crazyflie do:

```
make standard_config
```

then

```
make cf2
```

or

```
make bolt
```

