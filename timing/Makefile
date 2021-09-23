SRC = Heater.hs 

RUNH = runhaskell Heater.hs

all: copilot.c copilot.h

copilot.c: $(SRC) Heater.hs
	$(RUNH)

copilot.h: $(SRC) Heater.hs
	$(RUNH)

clean:
	rm -f copilot.* *~

commit:
	git commit -a -m="More Haskell Copilot work"

