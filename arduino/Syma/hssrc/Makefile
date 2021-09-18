#
# Makefile for Hackflight Copilot socket version
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

SRCDIR = ../../../source

SRC = Main.hs \
      $(SRCDIR)/Pin.hs \
      $(SRCDIR)/Safety.hs \
      $(SRCDIR)/Serial.hs \
      $(SRCDIR)/Receiver.hs \
      $(SRCDIR)/Hackflight.hs \
      $(SRCDIR)/HackflightFull.hs \
      $(SRCDIR)/Demands.hs \
      $(SRCDIR)/State.hs \
      $(SRCDIR)/Sensor.hs \
      $(SRCDIR)/Time.hs \
      $(SRCDIR)/Utils.hs \
      $(SRCDIR)/Mixer.hs \
      $(SRCDIR)/PidController.hs \
      $(SRCDIR)/pidcontrollers/RatePid.hs \
      $(SRCDIR)/pidcontrollers/YawPid.hs \
      $(SRCDIR)/pidcontrollers/LevelPid.hs \
      $(SRCDIR)/sensors/Gyrometer.hs \
      $(SRCDIR)/sensors/Quaternion.hs

RUNH = runhaskell -i$(SRCDIR) -i$(SRCDIR)/sensors -i$(SRCDIR)/pidcontrollers Main.hs

all: copilot.cpp copilot.h
	sed -i 's/extern/EXTERN/g' copilot.h
	cat $(SRCDIR)/cfiles/header.h copilot.h > ../copilot.h

copilot.cpp: copilot.c
	tail -n +8 copilot.c > tmp.c
	cat $(SRCDIR)/cfiles/header.c tmp.c > copilot.c
	rm tmp.c

copilot.c: $(SRC) Main.hs
	$(RUNH)

copilot.h: $(SRC) Main.hs
	$(RUNH)

clean:
	rm -f copilot.* tmp.c *~

commit:
	git commit -a -m="More Haskell Copilot work"

