SKETCH = motortest

PORT = /dev/ttyACM0

FQBN = teensy:avr:teensy40

build:
	arduino-cli compile --libraries $(HOME)/Documents/Arduino/libraries \
	   	--fqbn $(FQBN) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

upload:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3
