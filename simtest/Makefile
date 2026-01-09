all: simtest

run: simtest
	./simtest ../webots/worlds/simple.wbt ../webots/protos/DiyQuad.proto

simtest: main.o
	g++ -o simtest main.o

main.o: main.cpp
	g++ -Wall -c -I../.. -I../src main.cpp

edit:
	vim main.cpp

clean:
	rm -f simtest *.o
