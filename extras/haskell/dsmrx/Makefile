#
# Makefile for Hackflight Copilot socket version
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

HSRC = Main.hs

RUNH = runhaskell -i.. -i../sensors -i../pidcontrollers Main.hs

DST = /mnt/d/Users/levys/Documents/Arduino/libraries/Hackflight/examples/CopilotDsmrx/

all: copilot_dsmrx.h copilot_dsmrx.c

copilot_dsmrx.c: $(HSRC) Main.hs
	$(RUNH)

copilot_dsmrx.h: $(HSRC) Main.hs
	$(RUNH)

clean:
	rm -f tmp.c *copilot_dsmrx.* *~

commit:
	git commit -a -m="More Haskell Copilot work"

copy: copilot_dsmrx.cpp copilot_dsmrx.h
	cp copilot_dsmrx.c $(DST)/copilot_copilot_dsmrx.cpp
	cp copilot_dsmrx.h $(DST)/copilot_copilot_dsmrx.h

copilot_dsmrx.cpp: copilot_dsmrx.c
	tail -n +8 copilot_dsmrx.c > tmp.c
	cat ../header.c tmp.c > copilot_dsmrx.c
	rm tmp.c
