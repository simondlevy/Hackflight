INC = $(HOME)/Documents/Arduino/libraries/Hackflight/src/sensors

all: debug
	./debug

run: debug
	./debug

debug: debug.o
	g++ -o debug debug.o

debug.o: debug.cpp $(INC)/linalg.hpp
	g++ -I$(INC) -c debug.cpp

clean:
	rm -rf debug *.o *~
