SRC = Timer.hs 

RUNH = runhaskell Timer.hs

all: main

run: main
	./main

main: main.c timer.c timer.h
	gcc -o main main.c timer.c

timer.c: $(SRC) Timer.hs
	$(RUNH)

timer.h: $(SRC) Timer.hs
	$(RUNH)

clean:
	rm -f main timer.* *~

commit:
	git commit -a -m="More Haskell Copilot work"

