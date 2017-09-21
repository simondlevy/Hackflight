all: test_controller

test: test_controller
	./test_controller

test_controller: test_controller.o 
	g++ -o test_controller test_controller.o 

test_controller.o: test_controller.cpp controller.hpp
	g++ -c -Wall -I../.. test_controller.cpp

clean:
	rm -f test_controller *.o *~


