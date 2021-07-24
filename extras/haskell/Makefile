# Makefile for Haskell version of Hackflight
#
# Copyright (c) Simon D. Levy 2021
#
# MIT License

ALL = main
all: $(ALL)

WARN = -Wall

main: main.hs Hackflight.o VehicleState.o Demands.o ClosedLoopControl.o \
      Mixer.o Utils.o RatePid.o YawPid.o LevelPid.o AltHoldPid.o PosHoldPid.o \
      Server.o Sockets.o
	ghc -iclosedloops $(WARN) main.hs

Hackflight.o: Hackflight.hs VehicleState.o Demands.o Mixer.o
	ghc $(WARN) Hackflight.hs

VehicleState.o: VehicleState.hs
	ghc $(WARN) VehicleState.hs

Sensor.o: Sensor.hs VehicleState.o
	ghc $(WARN) Sensor.hs

Demands.o: Demands.hs
	ghc $(WARN) Demands.hs

OpenLoopControl.o: OpenLoopControl.hs Demands.o
	ghc $(WARN) OpenLoopControl.hs

ClosedLoopControl.o: ClosedLoopControl.hs Demands.o VehicleState.o
	ghc $(WARN) ClosedLoopControl.hs

Mixer.o: Mixer.hs
	ghc $(WARN) Mixer.hs

Server.o: Server.hs Hackflight.o Demands.o Mixer.o VehicleState.o Utils.o \
	  Sockets.o
	ghc $(WARN) Server.hs

SimReceiver.o: openloops/SimReceiver.hs OpenLoopControl.o Demands.o 
	ghc $(WARN) openloops/SimReceiver.hs
	mv openloops/SimReceiver.o .

SimSensor.o: sensors/SimSensor.hs VehicleState.o
	ghc $(WARN) sensors/SimSensor.hs
	mv sensors/SimSensor.o .

Sockets.o: Sockets.hs
	ghc $(WARN) Sockets.hs

Utils.o: Utils.hs
	ghc $(WARN) Utils.hs

RatePid.o: closedloops/RatePid.hs VehicleState.o ClosedLoopControl.o Demands.o \
	   Utils.o
	ghc $(WARN) closedloops/RatePid.hs
	mv closedloops/RatePid.o .

YawPid.o: closedloops/YawPid.hs VehicleState.o ClosedLoopControl.o Demands.o \
	  Utils.o
	ghc $(WARN) closedloops/YawPid.hs
	mv closedloops/YawPid.o .

LevelPid.o: closedloops/LevelPid.hs VehicleState.o ClosedLoopControl.o \
	    Demands.o Utils.o
	ghc $(WARN) closedloops/LevelPid.hs
	mv closedloops/LevelPid.o .

AltHoldPid.o: closedloops/AltHoldPid.hs \
	      VehicleState.o ClosedLoopControl.o Demands.o Utils.o
	ghc $(WARN) closedloops/AltHoldPid.hs
	mv closedloops/AltHoldPid.o .

PosHoldPid.o: closedloops/PosHoldPid.hs VehicleState.o ClosedLoopControl.o \
	      Demands.o Utils.o
	ghc $(WARN) closedloops/PosHoldPid.hs
	mv closedloops/PosHoldPid.o .

run: main
	./main
clean:
	rm -rf $(ALL) *.hi *.o */*.hi */*.o

commit:
	git commit -a -m="More Haskell work" && git push

