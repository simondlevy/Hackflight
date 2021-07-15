ALL = main KeyboardController
all: $(ALL)

main: main.hs \
      Hackflight.hs \
      Board.hs \
      Receiver.hs \
      Demands.hs \
      State.hs \
      Mixer.hs \
      Server.hs \
      PidController.hs \
      AltitudeController.hs
	ghc -Wall main.hs

KeyboardController: KeyboardController.hs
	ghc -Wall KeyboardController.hs

run: main
	./main

kb: KeyboardController
	./KeyboardController

clean:
	rm -rf $(ALL) *.hi *.o

edit:
	vim AltitudeController.hs

