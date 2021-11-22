/*
* Class declaration for thrust-vectoring rocket class using Haskell 
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

#include "RocketPawn.generated.h"

// Structures to hold static mesh initializations
DECLARE_STATIC_MESH(FFrameStatics, "Rocket/Body.Body", FrameStatics)

UCLASS(Config=Game)
class ARocketPawn : public APawn {

    private:

        GENERATED_BODY()

        FDynamicsThread * _dynamicsThread = NULL;

    protected:

        // AActor overrides

        virtual void BeginPlay() override;

        virtual void Tick(float DeltaSeconds) override;

        virtual void PostInitializeComponents() override;

        virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    public:	

        Vehicle vehicle;

        ARocketPawn();

}; // ARocketPawn
