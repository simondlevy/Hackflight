INCDIR = ../unreal/Source/HackflightSim/

all: test

test: tester
	./tester

tester: tester.o
	g++ -o tester tester.o

tester.o: tester.cpp 
	g++ -c -I$(INCDIR) tester.cpp

edit:
	vim tester.cpp

clean: 
	rm -f tester *.o *~


