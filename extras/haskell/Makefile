ALL = main
all: $(ALL)

WARN = -Wall

main: main.hs Hackflight.o State.o Demands.o Mixer.o PidControl.o \
      PidState.o Server.o Utils.o
	ghc $(WARN) main.hs
	rm -f *.hi *.o

main.o: main.hs Mixer.o Server.o PidControl.o
	ghc $(WARN) main.hs

Hackflight.o: Hackflight.hs State.o Demands.o Mixer.o PidControl.o
	ghc $(WARN) Hackflight.hs

PidState.o: PidState.hs
	ghc $(WARN) PidState.hs

State.o: State.hs
	ghc $(WARN) State.hs

Utils.o: Utils.hs
	ghc $(WARN) Utils.hs

Demands.o: Demands.hs
	ghc $(WARN) Demands.hs

Mixer.o: Mixer.hs
	ghc $(WARN) Mixer.hs

PidControl.o: PidControl.hs State.o Demands.o
	ghc $(WARN) PidControl.hs

Server.o: Server.hs Hackflight.o Demands.o Mixer.o State.o PidControl.o
	ghc $(WARN) Server.hs

run: main
	./main
clean:
	rm -rf $(ALL) *.hi *.o

commit:
	git commit -a -m="More Haskell work" && git push

