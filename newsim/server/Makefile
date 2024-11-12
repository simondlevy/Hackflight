HF_SRC = ../../src

ALL = server

all: $(ALL)

CFLAGS = -Wall -std=c++11

all: $(ALL)

server: server.o 
	g++ -o server server.o 

server.o: server.cpp
	g++ $(CFLAGS) -I$(HF_SRC) -Isockets -c server.cpp

run: server
	./server

edit:
	vim server.cpp

clean:
	rm -rf $(ALL) *.o *~
