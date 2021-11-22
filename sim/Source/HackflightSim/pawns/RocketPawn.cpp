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
