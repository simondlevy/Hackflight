# Copyright (c) 2021 Simon D. Levy
#
# MIT License

HSRC = Main.hs 

RUNH = runhaskell Main.hs

all: copilot.c copilot.h

copilot.c: $(HSRC) Main.hs
	$(RUNH)

copilot.h: $(HSRC) Main.hs
	$(RUNH)

clean:
	rm -f copilot.* *~
