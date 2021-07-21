ALL = main
all: $(ALL)

main: main.hs Hackflight.o State.o Demands.o Mixer.o PidControl.o \
      Server.o Utils.o
	ghc -Wall main.hs
	rm -f *.hi *.o

main.o: main.hs Mixer.o Server.o PidControl.o
	ghc -Wall main.hs

Hackflight.o: Hackflight.hs State.o Demands.o Mixer.o PidControl.o
	ghc -Wall Hackflight.hs

State.o: State.hs
	ghc -Wall State.hs

Utils.o: Utils.hs
	ghc -Wall Utils.hs

Demands.o: Demands.hs
	ghc -Wall Demands.hs

Mixer.o: Mixer.hs
	ghc -Wall Mixer.hs

PidControl.o: PidControl.hs State.o Demands.o
	ghc -Wall PidControl.hs

Server.o: Server.hs Hackflight.o Demands.o Mixer.o State.o PidControl.o
	ghc -Wall Server.hs

run: main
	./main
clean:
	rm -rf $(ALL) *.hi *.o

commit:
	git commit -a -m="More Haskell work" && git push

