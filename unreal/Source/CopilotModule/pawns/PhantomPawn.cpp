/*
* Class implementation for DJI Phantom pawn in MulticopterSim
*
* Copyright (C) 2018 Simon D. Levy
*
* MIT License
*/

#include "PhantomPawn.h"

APhantomPawn::APhantomPawn()
{
    _phantom.build(this);
}

void APhantomPawn::PostInitializeComponents()
{
    _phantom.PostInitializeComponents();

    Super::PostInitializeComponents();
}

// Called when the game starts or when spawned
void APhantomPawn::BeginPlay()
{
    _flightManager = new FCopilotFlightManager(this, &_phantom.dynamics);

    _phantom.BeginPlay(_flightManager);

    Super::BeginPlay();
}

void APhantomPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    _phantom.EndPlay();

    Super::EndPlay(EndPlayReason);
}

// Called automatically on main thread
void APhantomPawn::Tick(float DeltaSeconds)
{
    _phantom.Tick(DeltaSeconds);

    Super::Tick(DeltaSeconds);

    _flightManager->tick();
}
