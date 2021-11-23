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
DECLARE_STATIC_MESH(FFrameStatics, "Phantom/Frame.Frame", FrameStatics)
DECLARE_STATIC_MESH(FPropCWStatics, "Phantom/PropCW.PropCW", PropCWStatics)
DECLARE_STATIC_MESH(FPropCCWStatics, "Phantom/PropCCW.PropCCW", PropCCWStatics)

UCLASS(Config=Game)
class AIngenuityPawn : public APawn {

    private:

        GENERATED_BODY()

        FDynamicsThread * _dynamicsThread = NULL;

        void addRotor(UStaticMesh * mesh, int8_t dx, int8_t dy, int8_t dir);

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
