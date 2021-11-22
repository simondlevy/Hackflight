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
DECLARE_STATIC_MESH(FRotorTopStatics, "Rocket/RotorTop.RotorTop", RotorTopStatics)
DECLARE_STATIC_MESH(FRotorBottomStatics, "Rocket/RotorBottom.RotorBottom", RotorBottomStatics)
DECLARE_STATIC_MESH(FNozzleStatics, "Rocket/Nozzle.Nozzle", NozzleStatics)

UCLASS(Config=Game)
class ARocketPawn : public APawn {

    private:

        GENERATED_BODY()

        static constexpr double ROTOR_TOP_Z    =  0.70;
        static constexpr double ROTOR_BOTTOM_Z =  0.60;

        FDynamicsThread * _dynamicsThread = NULL;

        void addRotor(UStaticMesh* mesh, float z, int8_t dir);

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
