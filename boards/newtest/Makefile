hackflight.hex: hackflight.elf
	arm-none-eabi-objcopy -O ihex --set-start 0x8000000 hackflight.elf hackflight.hex

hackflight.elf: main.o
	#arm-none-eabi-g++ -std=gnu++11 -o hackflight.elf main.o -lc -specs=nosys.specs
	arm-none-eabi-g++ -o hackflight.elf main.o -lc -specs=nosys.specs

main.o: main.cpp mysimboard.hpp ../../newfirmware/*.hpp
	arm-none-eabi-g++ -std=gnu++11 -I../../newfirmware -c -o main.o main.cpp

clean:
	rm -f *.o *.hex *.elf *~
