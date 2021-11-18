HSDIR = ../haskell/

all: tester

test: tester
	./tester

run: tester
	./tester

tester: tester.o haskell.o
	g++ -o tester tester.o haskell.o

tester.o: tester.cpp Dynamics.hpp haskell.c
	g++ -c tester.cpp

haskell.o: haskell.c
	g++ -c haskell.c

haskell.c: Tester.hs Dynamics.hs
	runhaskell -i$(HSDIR) Tester.hs

edit:
	vim tester.cpp

clean: 
	rm -f tester haskell.* *.o *~
