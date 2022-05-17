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
    // Get height of barrel for dynamics
    float barrelHeight = meshHeightMeters(BodyStatics.mesh.Get());

    // Create vehicle body object
    vehicle.buildFull(this, BodyStatics.mesh.Get());

    // Add rotors
    addRotor(RotorTopStatics.mesh.Get(), ROTOR_TOP_Z, -1);
    addRotor(RotorBottomStatics.mesh.Get(), ROTOR_BOTTOM_Z, +1);

    // Add nozzle
    vehicle.nozzleMeshComponent =
        vehicle.addComponent(NozzleStatics.mesh.Get(),
                FName("Nozzle"), 0, 0, NozzleVehicle::NOZZLE_Z, 0);
}

float ARocketPawn::meshHeightMeters(UStaticMesh * mesh) 
{
    FBox box = mesh->GetBoundingBox();

    return (box.Max.Z - box.Min.Z) / 100; // cm => m
}

void ARocketPawn::PostInitializeComponents()
{
    vehicle.PostInitializeComponents();

    Super::PostInitializeComponents();
}

// Called when the game starts or when spawned
void ARocketPawn::BeginPlay()
{
    dynamicsThread = new FDynamicsThread(this);

    vehicle.BeginPlay(dynamicsThread);

    vehicle.pawn = this;

    Super::BeginPlay();
}

void ARocketPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    FDynamicsThread::stopThread(&dynamicsThread);

    Super::EndPlay(EndPlayReason);
}

// Called automatically on main thread
void ARocketPawn::Tick(float DeltaSeconds)
{
    vehicle.Tick(DeltaSeconds);

    Super::Tick(DeltaSeconds);

    dynamicsThread->tick();
}

void ARocketPawn::addRotor(UStaticMesh* mesh, float z, int8_t dir)
{
    vehicle.addRotor(mesh, 0, 0, z, dir);
}

void ARocketPawn::NozzleVehicle::animateActuators(void)
{
    Vehicle::animateActuators();

    float phi = pawn->dynamicsThread->getActuatorValue(2);
    float theta = pawn->dynamicsThread->getActuatorValue(3);

    nozzleMeshComponent->SetRelativeRotation(NOZZLE_MAX_ANGLE*FRotator(theta, 0, -phi));
}
