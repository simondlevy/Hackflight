all: simtest

run: simtest
	./simtest ../webots/worlds/simple.wbt

simtest: main.o
	g++ -o simtest main.o

main.o: main.cpp
	g++ -Wall -c -I../src main.cpp

edit:
	vim main.cpp

clean:
	rm -f simtest *.o
