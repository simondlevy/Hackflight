#
# Makefile for Hackflight sim version
#
# Copyright (c) 2021 on D. Levy
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
	rm -f copilot.* tmp.c *~
