#
# Makefile for Hackflight sim version
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

HSRC = Main.hs \
       HackflightFull.hs \
       Receiver.hs \
       Demands.hs \
       PidController.hs \
       Safety.hs \
       Time.hs \
       Mixer.hs \
       Utils.hs \
	   sensors/Gyrometer.hs \
	   sensors/Quaternion.hs \
       pidcontrollers/RatePid.hs \
       pidcontrollers/YawPid.hs \
       pidcontrollers/LevelPid.hs

RUNH = runhaskell -isensors -ipidcontrollers Main.hs

DSTDIR = ..

all: copilot.cpp copilot.h
	cp copilot.c $(DSTDIR)/copilot.cpp
	rm copilot.c
	sed -i 's/extern/EXTERN/g' copilot.h
	cat cfiles/header.h copilot.h > $(DSTDIR)/copilot.h

copilot.c: $(HSRC) Main.hs
	$(RUNH)

copilot.h: $(HSRC) Main.hs
	$(RUNH)

clean:
	rm -f copilot.* tmp.c *~

commit:
	git commit -a -m="More Haskell Copilot work"

copilot.cpp: copilot.c
	tail -n +8 copilot.c > tmp.c
	cat cfiles/header.c tmp.c > copilot.c
	rm tmp.c
