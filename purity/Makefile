test: test.o
	g++ -o test test.o

test.o: test.cpp
	g++ -Wall -I../src -c test.cpp

clean:
	rm -rf test test.o
