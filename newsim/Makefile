ALL = newsim

all: $(ALL)

CFLAGS = -Wall -std=c++11

all: $(ALL)

newsim: newsim.o 
	g++ -o newsim newsim.o 

newsim.o: main.cpp dynamics.hpp
	g++ $(CFLAGS) -o newsim.o -c main.cpp

run: newsim
	./newsim

edit:
	vim main.cpp

clean:
	rm -rf $(ALL) *.o *~
