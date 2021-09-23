SRC = Heater.hs 

RUNH = runhaskell Heater.hs

all: main

main: main.c heater.c heater.h
	gcc -o main main.c heater.c

heater.c: $(SRC) Heater.hs
	$(RUNH)

heater.h: $(SRC) Heater.hs
	$(RUNH)

clean:
	rm -f main heater.* *~

commit:
	git commit -a -m="More Haskell Copilot work"

