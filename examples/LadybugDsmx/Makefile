SKETCH = LadybugDsmx

FQBN = arduino-STM32L4:stm32l4:Ladybug-L432KC-FC

PORT = /dev/ttyACM0

DFU = obj/$(SKETCH).ino.dfu 

all: $(DFU)

$(DFU): $(SKETCH).ino 
	arduino-cli compile --fqbn $(FQBN) --libraries ../..,../../.. \
		--build-path obj --warnings "all"

upload: $(DFU)
	../../utils/dfu-touch.py $(PORT)
	$(HOME)/.arduino15/packages/arduino-STM32L4/stm32l4/tools/linux/stm32l4-upload \
	   	0x1209 0x6669 obj/$(SKETCH).ino.dfu 10
	rm -rf obj

unbrick: $(DFU)
	$(HOME)/.arduino15/packages/arduino-STM32L4/stm32l4/tools/linux/stm32l4-upload \
	   	0x1209 0x6669 obj/$(SKETCH).ino.dfu 10
	rm -rf obj

clean:
	rm -rf obj

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

