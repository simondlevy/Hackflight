ALL = Hackflight tmp.o
all: $(ALL)

Hackflight: Hackflight.hs Mixer.hs Server.hs PidControl.hs
	ghc -Wall Hackflight.hs
	rm -f *.hi *.o

Hackflight.o: Hackflight.hs Mixer.o Server.o PidControl.o
	ghc -Wall Hackflight.hs

State.o: State.hs
	ghc -Wall State.hs

Demands.o: Demands.hs
	ghc -Wall Demands.hs

Mixer.o: Mixer.hs
	ghc -Wall Mixer.hs

PidControl.o: PidControl.hs State.o Demands.o
	ghc -Wall PidControl.hs

Server.o: Server.hs Mixer.o State.o PidControl.o
	ghc -Wall Server.hs

tmp.o: tmp.hs 
	ghc -Wall -c tmp.hs

run: Hackflight
	./Hackflight
clean:
	rm -rf $(ALL) *.hi *.o

commit:
	git commit -a -m="More Haskell work" && git push

