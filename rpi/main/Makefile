ROOT = $(HOME)/Desktop

INCLUDE = -I$(ROOT)

ALL = main

all: $(ALL)

main: main.o
	g++ -O3 -o main main.o

main.o: main.cpp $(ROOT)/posix-utils/*.hpp \
	../../src/msp/parser.hpp ../../src/msp/serializer.hpp 
	g++ $(INCLUDE) -I../src -O3 -Wall -c main.cpp

run: main
	./main

clean:
	rm -f $(ALL) *.o *.hpp

edit:
	vim main.cpp
