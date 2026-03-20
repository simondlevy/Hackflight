all: main

main: main.cpp ../src/firmware/ekf/linalg.hpp
	g++ -I../src -I$(HOME)/Desktop/eigen -o main main.cpp

run: main
	./main

clean:
	rm -f main

edit:
	vim main.cpp

