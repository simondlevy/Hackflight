SKETCH = cf2

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

build: $(SKETCH).ino *.cpp
	arduino-cli compile \
	--libraries $(HOME)/Documents/Arduino/libraries --libraries ../.. \
	--build-path build --fqbn $(FQBN) $(SKETCH).ino

flash:
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/$(SKETCH).ino.bin

clean:
	rm -rf build

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

