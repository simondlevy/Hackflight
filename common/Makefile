all: readserial

read: readserial
	./readserial /dev/ttyUSB0

readserial: readserial.cpp serial.cpp serial.hpp
	g++ -o readserial readserial.cpp serial.cpp serial.hpp

clean:
	rm -f readserial
