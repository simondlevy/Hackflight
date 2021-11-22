/*
* Class implementation for thrust-vectoring rocket pawn in HackflightSim
*
* Copyright (C) 2018 Simon D. Levy
*
* MIT License
*/

#include "RocketPawn.h"

ARocketPawn::ARocketPawn()
{
    vehicle.buildFull(this, FrameStatics.mesh.Get());

    // Add rotors
    addRotor(RotorTopStatics.mesh.Get(), ROTOR_TOP_Z, -1);
    addRotor(RotorBottomStatics.mesh.Get(), ROTOR_BOTTOM_Z, +1);
}

void ARocketPawn::PostInitializeComponents()
{
    vehicle.PostInitializeComponents();

    Super::PostInitializeComponents();
}

// Called when the game starts or when spawned
void ARocketPawn::BeginPlay()
{
    _dynamicsThread = new FDynamicsThread(this);

    vehicle.BeginPlay(_dynamicsThread);

    Super::BeginPlay();
}

void ARocketPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    FDynamicsThread::stopThread(&_dynamicsThread);

    Super::EndPlay(EndPlayReason);
}

// Called automatically on main thread
void ARocketPawn::Tick(float DeltaSeconds)
{
    vehicle.Tick(DeltaSeconds);

    Super::Tick(DeltaSeconds);

    _dynamicsThread->tick();
}

void ARocketPawn::addRotor(UStaticMesh* mesh, float z, int8_t dir)
{
    vehicle.addRotor(mesh, 0, 0, z, dir);
}
