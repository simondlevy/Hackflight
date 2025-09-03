## Installation

1. Install Arduino and the Arduino Command-Line Interface

2. Install [STM32Duino](https://github.com/stm32duino/Arduino_Core_STM32)

3. Install these Arduino libraries:

* https://github.com/stm32duino/STM32FreeRTOS
* https://github.com/pololu/vl53l1x-arduino
* https://github.com/bolderflight/bmi088-arduino
* https://github.com/simondlevy/Arduino_CMSIS-DSP
* https://github.com/simondlevy/PMW3901
* https://github.com/simondlevy/OneShot125

4. In Arduino_CMSIS-DSP, do ```make install```

5. In hackflight/bolt/main do:

```make standard_config```

or


```make haskell_config```

6. Hold down the boot button on the Bolt for around five seconds, until the
blue LED blinks rapidly

```make && make flash```

