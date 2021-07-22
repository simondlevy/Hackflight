# Makefile for Haskell version of Hackflight
#
# Copyright (c) Simon D. Levy 2021
#
# MIT License

ALL = main
all: $(ALL)

WARN = -Wall

main: main.hs Hackflight.o VehicleState.o Sensor.o OpenLoopControl.o Demands.o \
      Mixer.o PidControl.o Server.o Utils.o \
      RatePid.o YawPid.o LevelPid.o AltHoldPid.o PosHoldPid.o
	ghc -ipids $(WARN) main.hs
	# rm -f *.hi *.o pids/*.hi pids/*.o

main.o: main.hs Mixer.o Server.o
	ghc $(WARN) main.hs

Hackflight.o: Hackflight.hs VehicleState.o Demands.o Mixer.o
	ghc $(WARN) Hackflight.hs

OpenLoopControl.o: OpenLoopControl.hs Demands.o
	ghc $(WARN) pids/OpenLoopControl.hs
	mv pids/OpenLoopControl.o .

Sensor.o: Sensor.hs VehicleState.o
	ghc $(WARN) pids/Sensor.hs
	mv pids/Sensor.o .

RatePid.o: pids/RatePid.hs VehicleState.o PidControl.o Demands.o Utils.o
	ghc $(WARN) pids/RatePid.hs
	mv pids/RatePid.o .

YawPid.o: pids/YawPid.hs VehicleState.o PidControl.o Demands.o Utils.o
	ghc $(WARN) pids/YawPid.hs
	mv pids/YawPid.o .

LevelPid.o: pids/LevelPid.hs VehicleState.o PidControl.o Demands.o Utils.o
	ghc $(WARN) pids/LevelPid.hs
	mv pids/LevelPid.o .

AltHoldPid.o: pids/AltHoldPid.hs VehicleState.o PidControl.o Demands.o Utils.o
	ghc $(WARN) pids/AltHoldPid.hs
	mv pids/AltHoldPid.o .

PosHoldPid.o: pids/PosHoldPid.hs VehicleState.o PidControl.o Demands.o Utils.o
	ghc $(WARN) pids/PosHoldPid.hs
	mv pids/PosHoldPid.o .

PidControl.o: PidControl.hs
	ghc $(WARN) PidControl.hs

VehicleState.o: VehicleState.hs
	ghc $(WARN) VehicleState.hs

Utils.o: Utils.hs
	ghc $(WARN) Utils.hs

Demands.o: Demands.hs
	ghc $(WARN) Demands.hs

Mixer.o: Mixer.hs
	ghc $(WARN) Mixer.hs

Server.o: Server.hs Hackflight.o Demands.o Mixer.o VehicleState.o
	ghc $(WARN) Server.hs

run: main
	./main
clean:
	rm -rf $(ALL) *.hi *.o pids/*.hi pids/*.o

commit:
	git commit -a -m="More Haskell work" && git push

