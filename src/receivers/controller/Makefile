all: tester

test: tester
	./tester

tester: tester.o 
	g++ -o tester tester.o 

tester.o: tester.cpp *.hpp
	g++ -c -Wall -I../.. tester.cpp

clean:
	rm -f tester *.o *~


