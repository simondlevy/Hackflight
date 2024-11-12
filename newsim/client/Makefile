ALL = client

all: $(ALL)

CFLAGS = -Wall -std=c++11

all: $(ALL)

client: client.o 
	g++ -o client client.o 

client.o: client.cpp dynamics.hpp
	g++ $(CFLAGS) -c client.cpp

run: client
	./client

edit:
	vim client.cpp

clean:
	rm -rf $(ALL) *.o *~
