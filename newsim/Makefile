all: newsim

run: newsim
	./newsim

newsim: main.cpp
	g++ -o newsim -I../src main.cpp

edit:
	vim main.cpp

plot: newsim
	./newsim > log.csv
	./plot.py

clean:
	rm -rf newsim
