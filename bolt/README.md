## Installation

1. Install [STM32Duino](https://github.com/stm32duino/Arduino_Core_STM32)

2. Install these Arduino libraries:

* https://github.com/simondlevy/Arduino_CMSIS-DSP
* https://github.com/simondlevy/PMW3901
* https://github.com/pololu/vl53l1x-arduino
* https://github.com/bolderflight/bmi088-arduino

3. In Arduino_CMSIS-DSP, do ```make install```

4. In hackflight/bolt/main do:

```make standard_config```

or


```make haskell_config```

5. Hold down the boot button on the Bolt for around five seconds, until the
blue LED blinks rapidly

```make && make flash```

