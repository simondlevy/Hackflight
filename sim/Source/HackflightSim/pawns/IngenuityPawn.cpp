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
    vehicle.buildFull(this, BodyStatics.mesh.Get());

    // Add propellers
    addRotor(PropCCWStatics.mesh.Get(), +1, +1, -1);
    addRotor(PropCCWStatics.mesh.Get(), -1, -1, -1);
    addRotor(PropCWStatics.mesh.Get(),  +1, -1, +1);
    addRotor(PropCWStatics.mesh.Get(),  -1, +1, +1);

    // Add mast, solar panel, antenna
    vehicle.addComponent(MastStatics.mesh.Get(), makeName("Mast", 1, "Mesh"));
    vehicle.addComponent(SolarPanelStatics.mesh.Get(), makeName("SolarPanel", 1, "Mesh"), 0, 0, 0.34);
    vehicle.addComponent(AntennaStatics.mesh.Get(), makeName("Antenna", 1, "Mesh"));

    // Add legs
    addLeg(1, Leg1BracketStatics.mesh.Get(), Leg1TopStatics.mesh.Get(), Leg1BottomStatics.mesh.Get());
    addLeg(2, Leg2BracketStatics.mesh.Get(), Leg2TopStatics.mesh.Get(), Leg2BottomStatics.mesh.Get());
    addLeg(3, Leg3BracketStatics.mesh.Get(), Leg3TopStatics.mesh.Get(), Leg3BottomStatics.mesh.Get());
    addLeg(4, Leg4BracketStatics.mesh.Get(), Leg4TopStatics.mesh.Get(), Leg4BottomStatics.mesh.Get());
}

void AIngenuityPawn::addLeg(uint8_t index, UStaticMesh * bracketMesh, UStaticMesh * topMesh, UStaticMesh * bottomMesh)
{
    vehicle.addComponent(bracketMesh, makeName("LegBracket", index, "Mesh"));
    vehicle.addComponent(topMesh,     makeName("LegTop", index, "Mesh"));
    vehicle.addComponent(bottomMesh,  makeName("LegBottom", index, "Mesh"));
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
