all: test

test: main.cpp parser.hpp
	g++ -o test main.cpp

run: test
	./test

clean:
	rm -f test *~
