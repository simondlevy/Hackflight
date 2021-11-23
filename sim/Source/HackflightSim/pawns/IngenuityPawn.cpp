/*
* Class implementation for NASA Ingenuity copter pawn in HackflightSim
*
* Copyright (C) 2018 Simon D. Levy
*
* MIT License
*/

#include "IngenuityPawn.h"

AIngenuityPawn::AIngenuityPawn()
{
    vehicle.buildFull(this, FrameStatics.mesh.Get());

    // Add propellers
    addRotor(PropCCWStatics.mesh.Get(), +1, +1, -1);
    addRotor(PropCCWStatics.mesh.Get(), -1, -1, -1);
    addRotor(PropCWStatics.mesh.Get(),  +1, -1, +1);
    addRotor(PropCWStatics.mesh.Get(),  -1, +1, +1);
}

void AIngenuityPawn::addRotor(UStaticMesh * mesh, int8_t dx, int8_t dy, int8_t dir)
{
    vehicle.addRotor(mesh, dx*0.12, dy*0.12, 0.16, dir);
}

void AIngenuityPawn::PostInitializeComponents()
{
    vehicle.PostInitializeComponents();

    Super::PostInitializeComponents();
}

// Called when the game starts or when spawned
void AIngenuityPawn::BeginPlay()
{
    _dynamicsThread = new FDynamicsThread(this);

    vehicle.BeginPlay(_dynamicsThread);

    Super::BeginPlay();
}

void AIngenuityPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    FDynamicsThread::stopThread(&_dynamicsThread);

    Super::EndPlay(EndPlayReason);
}

// Called automatically on main thread
void AIngenuityPawn::Tick(float DeltaSeconds)
{
    vehicle.Tick(DeltaSeconds);

    Super::Tick(DeltaSeconds);

    _dynamicsThread->tick();
}
