all: statetest rxtest motortest

statetest: statetest.cpp parser.hpp
	g++ -Wall -o statetest statetest.cpp

rxtest: rxtest.cpp parser.hpp
	g++ -Wall -o rxtest rxtest.cpp

motortest: motortest.cpp parser.hpp
	g++ -Wall -o motortest motortest.cpp

state: statetest
	./statetest

rx: rxtest
	./rxtest

motor: motortest
	./motortest

clean:
	rm -f test *~
