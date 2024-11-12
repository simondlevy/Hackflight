ALL = client

all: $(ALL)

CFLAGS = -Wall -std=c++11

all: $(ALL)

client: client.o 
	g++ -o client client.o 

client.o: main.cpp dynamics.hpp
	g++ $(CFLAGS) -o client.o -c main.cpp

run: client
	./client

edit:
	vim main.cpp

clean:
	rm -rf $(ALL) *.o *~
