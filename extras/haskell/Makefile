ALL = main tmp.o
all: $(ALL)

main: main.hs Mixer.hs Server.hs PidControl.hs
	ghc -Wall main.hs
	rm -f *.hi *.o

main.o: main.hs Mixer.o Server.o PidControl.o
	ghc -Wall main.hs

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

run: main
	./main
clean:
	rm -rf $(ALL) *.hi *.o

commit:
	git commit -a -m="More Haskell work" && git push

