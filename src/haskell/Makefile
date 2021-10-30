# Copyright (c) 2021 Simon D. Levy
#
# MIT License

HSRC = Ladybug.hs \
       Hackflight.hs \
       Receiver.hs \
       Demands.hs \
       PidController.hs \
       Safety.hs \
       Parser.hs \
       Serial.hs \
       Time.hs \
       Mixer.hs \
       Utils.hs \
	   sensors/Gyrometer.hs \
	   sensors/Quaternion.hs \
       pidcontrollers/RatePid.hs \
       pidcontrollers/YawPid.hs \
       pidcontrollers/LevelPid.hs

RUNH = runhaskell -isensors -ipidcontrollers Ladybug.hs

DSTDIR = ..

all: hackflight.cpp hackflight.h
	cp hackflight.c $(DSTDIR)/hackflight.cpp
	wc -c hackflight.c
	rm hackflight.c
	sed -i 's/extern/EXTERN/g' hackflight.h
	cat cfiles/header.h hackflight.h > $(DSTDIR)/hackflight.h

hackflight.c: $(HSRC) Ladybug.hs
	$(RUNH)

hackflight.h: $(HSRC) Ladybug.hs
	$(RUNH)

clean:
	rm -f hackflight.* ../hackflight.* *~

commit:
	git commit -a -m="More Haskell Copilot work"

hackflight.cpp: hackflight.c
	tail -n +8 hackflight.c > tmp.c
	cat cfiles/header.c tmp.c > hackflight.c
	rm tmp.c
