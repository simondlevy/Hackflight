all: statetest rxtest

statetest: statetest.cpp parser.hpp
	g++ -Wall -o statetest statetest.cpp

rxtest: rxtest.cpp parser.hpp
	g++ -Wall -o rxtest rxtest.cpp

state: statetest
	./statetest

rx: rxtest
	./rxtest

clean:
	rm -f test *~
