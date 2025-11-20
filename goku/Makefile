SKETCH = $(shell basename "`pwd`")

#FQBN = teensy:avr:teensy40
#FQBN = esp32:esp32:um_tinypico
FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

PORT = /dev/ttyACM0

BIN = build/$(SKETCH).ino.bin

$(BIN): $(SKETCH).ino
	arduino-cli compile --fqbn $(FQBN) $(SKETCH).ino --build-path ./build

build: $(SKETCH).ino
	arduino-cli compile --fqbn $(FQBN) $(SKETCH).ino

flash:
	echo 'R' > /dev/ttyACM0
	sleep 3
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)

dfu: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)

clean:
	rm -rf build

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3
