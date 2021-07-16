ALL = main
all: $(ALL)

main: main.hs Types.hs Multicopter.hs AltitudeController.hs
	ghc -Wall main.hs

run: main
	./main
clean:
	rm -rf $(ALL) *.hi *.o

edit:
	vim AltitudeController.hs

