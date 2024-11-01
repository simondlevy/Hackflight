all: newsim

run: newsim
	./newsim

newsim: main.cpp
	g++ -o newsim -I../src main.cpp

edit:
	vim main.cpp

csv:
	./newsim > log.csv
	libreoffice log.csv

clean:
	rm -rf newsim
