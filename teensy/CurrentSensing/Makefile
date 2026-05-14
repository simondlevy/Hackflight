SKETCH = $(shell basename "`pwd`")

FQBN = teensy:avr:teensy40
# FQBN = esp32:esp32:um_tinypico
#FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

PORT = /dev/ttyACM0

BIN = build/$(SKETCH).ino.bin

build: $(SKETCH).ino
	arduino-cli compile --fqbn $(FQBN) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

dfu: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)

clean:
	rm -rf obj

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3
