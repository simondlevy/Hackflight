/*
* Class declaration for NASA Ingenuity copter pawn class using Haskell 
*
* Copyright (C) 2021 Simon D. Levy
*
* MIT License
*/

#pragma once

#include "../Vehicle.hpp"
#include "../DynamicsThread.h"

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "IngenuityPawn.generated.h"

// Structures to hold static mesh initializations

DECLARE_STATIC_MESH(FPropCWStatics, "Phantom/PropCW.PropCW", PropCWStatics)
DECLARE_STATIC_MESH(FPropCCWStatics, "Phantom/PropCCW.PropCCW", PropCCWStatics)

DECLARE_STATIC_MESH(FBodyStatics, "Ingenuity/Body.Body", BodyStatics)
DECLARE_STATIC_MESH(FRotorBottomStatics, "Ingenuity/Rotor_Bottom.Rotor_Bottom", RotorBottomStatics)
DECLARE_STATIC_MESH(FRotorTopStatics, "Ingenuity/Rotor_Top.Rotor_Top", RotorTopStatics)
DECLARE_STATIC_MESH(FMastStatics, "Ingenuity/Mast.Mast", MastStatics)
DECLARE_STATIC_MESH(FSolar_PanelStatics, "Ingenuity/Solar_Panel.Solar_Panel", SolarPanelStatics)
DECLARE_STATIC_MESH(FAntennaStatics, "Ingenuity/Antenna.Antenna", AntennaStatics)

DECLARE_STATIC_MESH(FLeg1BottomStatics, "Ingenuity/Leg1_Bottom.Leg1_Bottom", Leg1BottomStatics)
DECLARE_STATIC_MESH(FLeg1BracketStatics, "Ingenuity/Leg1_Bracket.Leg1_Bracket", Leg1BracketStatics)
DECLARE_STATIC_MESH(FLeg1TopStatics, "Ingenuity/Leg1_Top.Leg1_Top", Leg1TopStatics)

DECLARE_STATIC_MESH(FLeg2BottomStatics, "Ingenuity/Leg2_Bottom.Leg2_Bottom", Leg2BottomStatics)
DECLARE_STATIC_MESH(FLeg2BracketStatics, "Ingenuity/Leg2_Bracket.Leg2_Bracket", Leg2BracketStatics)
DECLARE_STATIC_MESH(FLeg2TopStatics, "Ingenuity/Leg2_Top.Leg2_Top", Leg2TopStatics)

DECLARE_STATIC_MESH(FLeg3BottomStatics, "Ingenuity/Leg3_Bottom.Leg3_Bottom", Leg3BottomStatics)
DECLARE_STATIC_MESH(FLeg3BracketStatics, "Ingenuity/Leg3_Bracket.Leg3_Bracket", Leg3BracketStatics)
DECLARE_STATIC_MESH(FLeg3TopStatics, "Ingenuity/Leg3_Top.Leg3_Top", Leg3TopStatics)

DECLARE_STATIC_MESH(FLeg4BottomStatics, "Ingenuity/Leg4_Bottom.Leg4_Bottom", Leg4BottomStatics)
DECLARE_STATIC_MESH(FLeg4BracketStatics, "Ingenuity/Leg4_Bracket.Leg4_Bracket", Leg4BracketStatics)
DECLARE_STATIC_MESH(FLeg4TopStatics, "Ingenuity/Leg4_Top.Leg4_Top", Leg4TopStatics)

UCLASS(Config=Game)
class AIngenuityPawn : public APawn {

    private:

        GENERATED_BODY()

        FDynamicsThread * _dynamicsThread = NULL;

        void addRotor(UStaticMesh * mesh, int8_t dx, int8_t dy, int8_t dir);

        void addLeg(uint8_t index, UStaticMesh * bracketMesh, UStaticMesh * topMesh, UStaticMesh * bottomMesh);

    protected:

        // AActor overrides

        virtual void BeginPlay() override;

        virtual void Tick(float DeltaSeconds) override;

        virtual void PostInitializeComponents() override;

        virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    public:	

        Vehicle vehicle;

        AIngenuityPawn();

}; // AIngenuityPawn
