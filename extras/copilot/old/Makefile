all: heater array

heater: heater_main.c heater.c heater.h
	gcc -Wall -std=c99 heater.c heater_main.c -o heater

heater.c: Heater.hs
	runhaskell Heater.hs

heater.h: Heater.hs
	runhaskell Heater.hs

array: array_main.c array.c array.h
	gcc -Wall -std=c99 array.c array_main.c -o array

array.c: Array.hs
	runhaskell Array.hs

array.h: Array.hs
	runhaskell Array.hs

clean:
	rm -f heater heater.c heater.h array array.c array.h
