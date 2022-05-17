/*
* Class implementation for DJI Phantom pawn in HackflightSim
*
* Copyright (C) 2018 Simon D. Levy
*
* MIT License
*/

#include "PhantomPawn.h"

APhantomPawn::APhantomPawn()
{
    vehicle.buildFull(this, FrameStatics.mesh.Get());

    // Add propellers
    addRotor(PropCCWStatics.mesh.Get(), +1, +1, -1);
    addRotor(PropCCWStatics.mesh.Get(), -1, -1, -1);
    addRotor(PropCWStatics.mesh.Get(),  +1, -1, +1);
    addRotor(PropCWStatics.mesh.Get(),  -1, +1, +1);
}

void APhantomPawn::addRotor(UStaticMesh * mesh, int8_t dx, int8_t dy, int8_t dir)
{
    vehicle.addRotor(mesh, dx*0.12, dy*0.12, 0.16, dir);
}

void APhantomPawn::PostInitializeComponents()
{
    vehicle.PostInitializeComponents();

    Super::PostInitializeComponents();
}

// Called when the game starts or when spawned
void APhantomPawn::BeginPlay()
{
    _dynamicsThread = new FDynamicsThread(this);

    vehicle.BeginPlay(_dynamicsThread);

    Super::BeginPlay();
}

void APhantomPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    FDynamicsThread::stopThread(&_dynamicsThread);

    Super::EndPlay(EndPlayReason);
}

// Called automatically on main thread
void APhantomPawn::Tick(float DeltaSeconds)
{
    vehicle.Tick(DeltaSeconds);

    Super::Tick(DeltaSeconds);

    _dynamicsThread->tick();
}
