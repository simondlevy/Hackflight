INCLUDE_DIRS = lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x \
               lib/STM32F10x_StdPeriph_Driver/inc \
			   lib/CMSIS/CM3/CoreSupport \
			   .

CFLAGS = $(addprefix -I, $(INCLUDE_DIRS)) -DSTM32F10X_MD \

hackflight.hex: hackflight.elf
	arm-none-eabi-objcopy -O ihex --set-start 0x8000000 hackflight.elf hackflight.hex

hackflight.elf: main.o
	arm-none-eabi-g++ -o hackflight.elf main.o -lc -specs=nosys.specs

main.o: main.cpp naze.hpp ../../newfirmware/*.hpp
	arm-none-eabi-g++ $(CFLAGS) -std=gnu++11 -I../../newfirmware -c -o main.o main.cpp

clean:
	rm -f *.o *.hex *.elf *~
