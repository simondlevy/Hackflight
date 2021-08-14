heater: main.c heater.c heater.h
	gcc -Wall -std=c99 heater.c main.c -o heater

heater.c: Heater.hs
	runhaskell Heater.hs

heater.h: Heater.hs
	runhaskell Heater.hs

clean:
	rm -f heater heater.c heater.h

run: heater
	./heater



