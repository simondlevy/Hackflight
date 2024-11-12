HF_SRC = ../../src

ALL = server

all: $(ALL)

CFLAGS = -Wall -std=c++11

all: $(ALL)

server: server.o 
	g++ -o server server.o 

server.o: main.cpp
	g++ $(CFLAGS) -I$(HF_SRC) -Isockets -o server.o -c main.cpp

run: server
	./server

edit:
	vim main.cpp

clean:
	rm -rf $(ALL) *.o *~
