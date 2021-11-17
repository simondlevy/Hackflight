CPPDIR = ../unreal/Source/HackflightSim/
HSDIR = ../haskell/

all: test

test: tester
	./tester

tester: tester.o
	g++ -o tester tester.o

tester.o: tester.cpp $(CPPDIR)/Dynamics.hpp $(HSDIR)/Dynamics.hs
	g++ -c -I$(CPPDIR) tester.cpp

edit:
	vim tester.cpp

clean: 
	rm -f tester *.o *~


