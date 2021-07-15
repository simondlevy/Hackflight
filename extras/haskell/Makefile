ALL = takeoff
all: $(ALL)

takeoff: takeoff.hs Types.hs Multicopter.hs AltitudeController.hs
	ghc -Wall takeoff.hs

run: takeoff
	./takeoff
clean:
	rm -rf $(ALL) *.hi *.o

edit:
	vim AltitudeController.hs

