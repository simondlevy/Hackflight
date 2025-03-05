SKETCH = test

FQBN = STMicroelectronics:stm32:GenF4:pnum=CF2_F405

PORT = /dev/ttyACM0

BIN = build/$(SKETCH).ino.bin

$(BIN): $(SKETCH).ino *.cpp *.h
	arduino-cli compile --fqbn $(FQBN)  --build-path=$(PWD)/build $(SKETCH).ino

flash: $(BIN)
	./usb-bootloader.py
	dfu-util -d 0483:df11 -a 0 -s 0x08004000:leave -D $(BIN)

clean:
	rm -rf build

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200
