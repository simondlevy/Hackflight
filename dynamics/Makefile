CPPDIR = ../unreal/Source/HackflightSim/
HSDIR = ../haskell/

all: tester

test: tester
	./tester

tester: tester.o haskell.o
	g++ -o tester tester.o haskell.o

tester.o: tester.cpp $(CPPDIR)/Dynamics.hpp 
	g++ -c -I$(CPPDIR) tester.cpp

haskell.o: haskell.c
	g++ -c haskell.c

haskell.c: Tester.hs $(HSDIR)/Dynamics.hs
	runhaskell -i$(HSDIR) Tester.hs

edit:
	vim tester.cpp

clean: 
	rm -f tester haskell.* *.o *~

commit:
	git commit -a -m="More dynamics testing" && gitpush hackflight
