ALL = takeoff
all: $(ALL)

takeoff: takeoff.hs Demands.hs State.hs Mixer.hs Server.hs PidController.hs AltitudeController.hs
	ghc -Wall takeoff.hs

run: takeoff
	./takeoff
clean:
	rm -rf $(ALL) *.hi *.o

edit:
	vim AltitudeController.hs

