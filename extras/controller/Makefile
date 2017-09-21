all: test_controller

test: test_controller
	./test_controller

test_controller: test_controller.o controller.o
	g++ -o test_controller test_controller.o controller.o

test_controller.o: test_controller.cpp ../../src/*.hpp
	g++ -c -Wall test_controller.cpp

controller.o: controller.cpp ../../src/*.hpp
	g++ -c -Wall controller.cpp

clean:
	rm -f test_controller *.o *~


