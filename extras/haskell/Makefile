ALL = Hackflight
all: $(ALL)

Hackflight: Hackflight.hs Mixer.hs Server.hs AltHoldPid.hs
	ghc -Wall Hackflight.hs

run: Hackflight
	./Hackflight
clean:
	rm -rf $(ALL) *.hi *.o

edit:
	vim AltHoldPid.hs

commit:
	git commit -m="More Haskell work" && git push

