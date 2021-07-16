ALL = main
all: $(ALL)

main: main.hs Mixer.hs Multicopter.hs AltHoldPid.hs
	ghc -Wall main.hs

run: main
	./main
clean:
	rm -rf $(ALL) *.hi *.o

edit:
	vim AltHoldPid.hs

