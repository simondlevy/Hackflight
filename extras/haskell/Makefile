ALL = main
all: $(ALL)

WARN = -Wall

main: main.hs Hackflight.o VehicleState.o Demands.o Mixer.o PidControllers.o \
      PidControl.o Server.o Utils.o
	ghc $(WARN) main.hs
	rm -f *.hi *.o

main.o: main.hs Mixer.o Server.o PidControllers.o
	ghc $(WARN) main.hs

Hackflight.o: Hackflight.hs VehicleState.o Demands.o Mixer.o PidControllers.o
	ghc $(WARN) Hackflight.hs

PidControl.o: PidControl.hs
	ghc $(WARN) PidControl.hs

VehicleState.o: VehicleState.hs
	ghc $(WARN) VehicleState.hs

Utils.o: Utils.hs
	ghc $(WARN) Utils.hs

Demands.o: Demands.hs
	ghc $(WARN) Demands.hs

Mixer.o: Mixer.hs
	ghc $(WARN) Mixer.hs

PidControllers.o: PidControllers.hs VehicleState.o Demands.o
	ghc $(WARN) PidControllers.hs

Server.o: Server.hs Hackflight.o Demands.o Mixer.o VehicleState.o PidControllers.o
	ghc $(WARN) Server.hs

run: main
	./main
clean:
	rm -rf $(ALL) *.hi *.o

commit:
	git commit -a -m="More Haskell work" && git push

