all: newsim

run: newsim
	./newsim

newsim: main.cpp
	g++ -o newsim -I../src main.cpp

edit:
	vim main.cpp

clean:
	rm -rf newsim
