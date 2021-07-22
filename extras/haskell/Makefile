# Makefile for Haskell version of Hackflight
#
# Copyright (c) Simon D. Levy 2021
#
# MIT License

ALL = main
all: $(ALL)

WARN = -Wall

main: main.hs Hackflight.o VehicleState.o Sensor.o Demands.o OpenLoopControl.o \
	  ClosedLoopControl.o Mixer.o PidControl.o Server.o Utils.o \
	  PassThruSensor.o \
      RatePid.o YawPid.o LevelPid.o AltHoldPid.o PosHoldPid.o
	ghc -ipids -isensors $(WARN) main.hs

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

PidControl.o: PidControl.hs
	ghc $(WARN) PidControl.hs

Server.o: Server.hs Hackflight.o Demands.o Mixer.o VehicleState.o
	ghc $(WARN) Server.hs

Utils.o: Utils.hs
	ghc $(WARN) Utils.hs

PassThruSensor.o: sensors/PassThruSensor.hs Sensor.o
	ghc $(WARN) sensors/PassThruSensor.hs
	mv sensors/PassThruSensor.o .

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

run: main
	./main
clean:
	rm -rf $(ALL) *.hi *.o pids/*.hi pids/*.o

commit:
	git commit -a -m="More Haskell work" && git push

