all:
	~/Development/arduino-1.6.8/arduino --verify hackflight.ino

flash:
	~/Development/arduino-1.6.8/arduino --upload hackflight.ino

listen:
	miniterm.py /dev/ttyACM1 115200

edit:
	vim board_teensy.cpp
