all: main

main: main.cpp ../src/firmware/ekf/linalg.hpp
	gcc -I../src -o main main.cpp

run: main
	./main

clean:
	rm -f main

edit:
	vim main.cpp

