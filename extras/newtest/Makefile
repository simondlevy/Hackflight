# Copyright (c) 2021 Simon D. Levy
#
# MIT License

HSRC = Main.hs 

RUNH = runhaskell Main.hs

all: main

run: main
	./main

main: main.c copilot.c copilot.h
	gcc -o main main.c copilot.c

copilot.c: $(HSRC) Main.hs
	$(RUNH)

copilot.h: $(HSRC) Main.hs
	$(RUNH)

clean:
	rm -f main copilot.* *~
