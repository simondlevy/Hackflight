all: test

test: main.cpp parser.hpp
	g++ -o test main.cpp

run: test
	./test

count: test
	./test | wc -lc

clean:
	rm -f test *~
