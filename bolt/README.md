## Installation

1. Install [STM32Duino](https://github.com/stm32duino/Arduino_Core_STM32)

2. Install these Arduino libraries:

* https://github.com/simondlevy/Arduino_CMSIS-DSP
* https://github.com/simondlevy/PMW3901
* https://github.com/simondlevy/VL53L1

3. In Arduino_CMSIS-DSP, do ```make install```

4. In hackflight/bolt do:


```make standard_config```

or


```make haskell_config```

then

```make -j 32```

