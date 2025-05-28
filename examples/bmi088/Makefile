SKETCH = bmi088

# FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen
FQBN = teensy:avr:teensy40

# PORT = /dev/ttyACM0
PORT = COM3

LIBS = $(HOME)/Documents/Arduino/libraries

build: $(SKETCH).ino Makefile
	arduino-cli compile --libraries $(LIBS) --fqbn $(FQBN) $(SKETCH).ino

flash: $(BIN)
	#dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)
	arduino-cli upload -p $(PORT) --fqbn $(FQBN)

clean:
	rm -rf build

edit:
	vim $(SKETCH).ino

listen:
	#miniterm.py $(PORT) 115200 --exit-char 3 # exit on CTRL-C
	plink -serial $(PORT)

