ALL = Hackflight
all: $(ALL)

Hackflight: Hackflight.hs Mixer.hs Server.hs AltHoldPid.hs
	ghc -Wall Hackflight.hs
	rm -f *.hi *.o

run: Hackflight
	./Hackflight
clean:
	rm -rf $(ALL) *.hi *.o

commit:
	git commit -a -m="More Haskell work" && git push

