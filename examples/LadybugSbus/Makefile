SKETCH = LadybugSbus

FQBN = arduino-STM32L4:stm32l4:Ladybug-L432KC-FC

PORT = /dev/ttyACM0

HFLIB = ../../
LIB = ../../..
DFU = obj/$(SKETCH).ino.dfu 
SRC = $(HFLIB)/src

all: $(DFU)

$(DFU): $(SKETCH).ino $(SRC)/*.h $(SRC)/*/*.h
	arduino-cli compile --fqbn $(FQBN) --libraries $(HFLIB),$(LIB) --build-path obj --warnings "all"

upload:
	../../utils/dfu-touch.py $(PORT)
	$(HOME)/.arduino15/packages/arduino-STM32L4/stm32l4/tools/linux/stm32l4-upload \
	   	0x1209 0x6669 obj/$(SKETCH).ino.dfu 10

unbrick:
	$(HOME)/.arduino15/packages/arduino-STM32L4/stm32l4/tools/linux/stm32l4-upload \
	   	0x1209 0x6669 obj/$(SKETCH).ino.dfu 10

clean:
	rm -rf obj

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

