all: test

test: test.cpp parser.hpp
	g++ -o test test.cpp

run: test
	./test

clean:
	rm -f test *~
