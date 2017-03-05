newtest: main.cpp ../../newfirmware/*.hpp
	g++ -std=gnu++11 -I../../newfirmware -o newtest main.cpp 

run: newtest
	./newtest

edit:
	vim mysimboard.hpp

clean:
	rm -f newtest *~
